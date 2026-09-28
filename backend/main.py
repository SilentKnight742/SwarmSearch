from contextlib import asynccontextmanager

from fastapi import (
    FastAPI,
    HTTPException,
    WebSocket,
    WebSocketDisconnect,
)
from fastapi.middleware.cors import (
    CORSMiddleware,
)
from pydantic import (
    BaseModel,
    Field,
)

from .event_broker import EventBroker
from .runtime import MissionRuntime


broker = EventBroker()

runtime = MissionRuntime(
    broker=broker
)


@asynccontextmanager
async def lifespan(
    app: FastAPI,
):
    broker.start()

    yield

    await runtime.shutdown()

    broker.stop()


app = FastAPI(
    title=(
        "SwarmSearch Mission Control"
    ),
    version="0.1.0",
    lifespan=lifespan,
)


app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173",
        "http://127.0.0.1:5173",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ======================================================
# REQUEST MODELS
# ======================================================


class MissionStartRequest(
    BaseModel
):
    width_m: float = Field(
        default=60.0,
        gt=0,
        le=1000,
    )

    height_m: float = Field(
        default=30.0,
        gt=0,
        le=1000,
    )

    lane_spacing_m: float = Field(
        default=15.0,
        gt=0,
        le=250,
    )


class FailureRequest(
    BaseModel
):
    reason: str = Field(
        default=(
            "operator-injected failure"
        ),
        min_length=1,
        max_length=200,
    )


# ======================================================
# HTTP
# ======================================================


@app.get(
    "/api/health"
)
async def health():
    return {
        "status": "ok",
        "service": (
            "swarmsearch-mission-control"
        ),
        **runtime.status(),
    }


@app.get(
    "/api/state"
)
async def state():
    return runtime.snapshot()


@app.post(
    "/api/mission/start",
    status_code=202,
)
async def start_mission(
    request: MissionStartRequest,
):
    try:
        await runtime.start_mission(
            width_m=request.width_m,
            height_m=request.height_m,
            lane_spacing_m=(
                request.lane_spacing_m
            ),
        )

    except RuntimeError as exc:
        raise HTTPException(
            status_code=409,
            detail=str(exc),
        ) from exc

    return {
        "accepted": True,
        **runtime.status(),
        "mission": {
            "width_m": request.width_m,
            "height_m": request.height_m,
            "lane_spacing_m": (
                request.lane_spacing_m
            ),
        },
    }


@app.post(
    "/api/mission/failure/{vehicle_name}",
)
async def inject_failure(
    vehicle_name: str,
    request: FailureRequest,
):
    try:
        runtime.inject_failure(
            vehicle_name=vehicle_name,
            reason=request.reason,
        )

    except KeyError as exc:
        raise HTTPException(
            status_code=404,
            detail=str(exc),
        ) from exc

    except RuntimeError as exc:
        raise HTTPException(
            status_code=409,
            detail=str(exc),
        ) from exc

    return {
        "accepted": True,
        "vehicle": vehicle_name,
        "reason": request.reason,
    }


# ======================================================
# WEBSOCKET
# ======================================================


@app.websocket(
    "/ws"
)
async def websocket_endpoint(
    websocket: WebSocket,
):
    await websocket.accept()

    queue = broker.subscribe()

    try:
        await websocket.send_json(
            {
                "type": "snapshot",
                "data": (
                    runtime.snapshot()
                ),
            }
        )

        while True:
            event = await queue.get()

            await websocket.send_json(
                {
                    "type": "event",
                    "runtime": (
                        runtime.status()
                    ),
                    "data": event,
                }
            )

    except WebSocketDisconnect:
        pass

    finally:
        broker.unsubscribe(
            queue
        )