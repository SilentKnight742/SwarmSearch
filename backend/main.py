import asyncio

from contextlib import (
    asynccontextmanager,
)

from fastapi import (
    FastAPI,
    Header,
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

from .event_broker import (
    EventBroker,
)

from .runtime import (
    MissionRuntime,
)

from .session_manager import (
    QueueFullError,
    SessionManager,
    SessionPermissionError,
    SessionStateError,
)


SESSION_HEADER = (
    "X-SwarmSearch-Session"
)


broker = EventBroker()

runtime = MissionRuntime(
    broker=broker
)

sessions = SessionManager(
    on_active_release=(
        runtime.release_session
    ),
    max_queue_size=10,
    queue_heartbeat_timeout=60,
    active_idle_timeout=15 * 60,
    active_max_duration=30 * 60,
    grant_timeout=60,
)


@asynccontextmanager
async def lifespan(
    app: FastAPI,
):
    broker.start()

    await sessions.start()

    yield

    await sessions.stop()

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


def session_token(
    value: str | None,
) -> str | None:
    return value


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


# ======================================================
# PUBLIC SIMULATOR SESSION
# ======================================================


@app.get(
    "/api/session"
)
async def get_session(
    x_session: str | None = Header(
        default=None,
        alias=SESSION_HEADER,
    ),
):
    return await sessions.status(
        x_session
    )


@app.post(
    "/api/session/request"
)
async def request_session(
    x_session: str | None = Header(
        default=None,
        alias=SESSION_HEADER,
    ),
):
    try:
        return await sessions.request(
            x_session
        )

    except QueueFullError as exc:
        raise HTTPException(
            status_code=429,
            detail=str(exc),
        ) from exc


@app.post(
    "/api/session/heartbeat"
)
async def heartbeat_session(
    x_session: str | None = Header(
        default=None,
        alias=SESSION_HEADER,
    ),
):
    if not x_session:
        raise HTTPException(
            status_code=400,
            detail=(
                "Session token required."
            ),
        )

    return await sessions.heartbeat(
        x_session
    )


@app.post(
    "/api/session/ack"
)
async def acknowledge_session(
    x_session: str | None = Header(
        default=None,
        alias=SESSION_HEADER,
    ),
):
    if not x_session:
        raise HTTPException(
            status_code=400,
            detail=(
                "Session token required."
            ),
        )

    try:
        return await (
            sessions.acknowledge(
                x_session
            )
        )

    except SessionStateError as exc:
        raise HTTPException(
            status_code=409,
            detail=str(exc),
        ) from exc


@app.post(
    "/api/session/release"
)
async def release_session(
    x_session: str | None = Header(
        default=None,
        alias=SESSION_HEADER,
    ),
):
    if not x_session:
        raise HTTPException(
            status_code=400,
            detail=(
                "Session token required."
            ),
        )

    return await sessions.release(
        x_session
    )


# ======================================================
# MISSION CONTROL
# ======================================================


@app.post(
    "/api/mission/start",
    status_code=202,
)
async def start_mission(
    request: MissionStartRequest,
    x_session: str | None = Header(
        default=None,
        alias=SESSION_HEADER,
    ),
):
    try:
        await sessions.require_active(
            x_session
        )

    except SessionPermissionError as exc:
        raise HTTPException(
            status_code=403,
            detail=str(exc),
        ) from exc

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
            "width_m": (
                request.width_m
            ),
            "height_m": (
                request.height_m
            ),
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
    x_session: str | None = Header(
        default=None,
        alias=SESSION_HEADER,
    ),
):
    try:
        await sessions.require_active(
            x_session
        )

    except SessionPermissionError as exc:
        raise HTTPException(
            status_code=403,
            detail=str(exc),
        ) from exc

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
    token = (
        websocket.query_params.get(
            "session_token"
        )
    )

    await websocket.accept()

    mission_queue = (
        broker.subscribe()
    )

    session_queue = (
        await sessions.subscribe(
            token
        )
    )

    mission_waiter = (
        asyncio.create_task(
            mission_queue.get()
        )
    )

    session_waiter = (
        asyncio.create_task(
            session_queue.get()
        )
    )

    try:
        await websocket.send_json(
            {
                "type": "snapshot",
                "data": (
                    runtime.snapshot()
                ),
                "session": (
                    await sessions.status(
                        token
                    )
                ),
            }
        )

        while True:
            done, _ = (
                await asyncio.wait(
                    {
                        mission_waiter,
                        session_waiter,
                    },
                    return_when=(
                        asyncio.FIRST_COMPLETED
                    ),
                )
            )

            if (
                mission_waiter
                in done
            ):
                event = (
                    mission_waiter.result()
                )

                await websocket.send_json(
                    {
                        "type": "event",
                        "runtime": (
                            runtime.status()
                        ),
                        "data": event,
                    }
                )

                mission_waiter = (
                    asyncio.create_task(
                        mission_queue.get()
                    )
                )

            if (
                session_waiter
                in done
            ):
                session_status = (
                    session_waiter.result()
                )

                await websocket.send_json(
                    {
                        "type": "session",
                        "data": (
                            session_status
                        ),
                    }
                )

                session_waiter = (
                    asyncio.create_task(
                        session_queue.get()
                    )
                )

    except WebSocketDisconnect:
        pass

    finally:
        mission_waiter.cancel()
        session_waiter.cancel()

        broker.unsubscribe(
            mission_queue
        )

        await sessions.unsubscribe(
            token,
            session_queue,
        )