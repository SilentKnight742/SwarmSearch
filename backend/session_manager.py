import asyncio
import secrets
import time

from collections import deque
from collections.abc import (
    Awaitable,
    Callable,
)
from dataclasses import dataclass
from typing import Optional


class QueueFullError(
    RuntimeError
):
    pass


class SessionPermissionError(
    RuntimeError
):
    pass


class SessionStateError(
    RuntimeError
):
    pass


@dataclass
class QueueEntry:
    token: str
    joined_at: float
    last_seen: float


ReleaseCallback = Callable[
    [],
    Awaitable[None],
]


class SessionManager:
    def __init__(
        self,
        on_active_release: Optional[
            ReleaseCallback
        ] = None,
        max_queue_size: int = 10,
        queue_heartbeat_timeout: float = 60.0,
        active_idle_timeout: float = 15 * 60,
        active_max_duration: float = 30 * 60,
        grant_timeout: float = 60.0,
    ):
        self._on_active_release = (
            on_active_release
        )

        self.max_queue_size = (
            max_queue_size
        )

        self.queue_heartbeat_timeout = (
            queue_heartbeat_timeout
        )

        self.active_idle_timeout = (
            active_idle_timeout
        )

        self.active_max_duration = (
            active_max_duration
        )

        self.grant_timeout = (
            grant_timeout
        )

        self._active_token: Optional[
            str
        ] = None

        self._active_started_at: Optional[
            float
        ] = None

        self._active_last_activity: Optional[
            float
        ] = None

        self._active_last_seen: Optional[
            float
        ] = None

        self._pending_token: Optional[
            str
        ] = None

        self._pending_expires_at: Optional[
            float
        ] = None

        self._queue: deque[
            QueueEntry
        ] = deque()

        self._releasing = False

        self._lock = asyncio.Lock()

        self._cleanup_task: Optional[
            asyncio.Task
        ] = None

        self._subscribers: dict[
            Optional[str],
            set[asyncio.Queue],
        ] = {}

    # ==================================================
    # LIFECYCLE
    # ==================================================

    async def start(
        self,
    ):
        if (
            self._cleanup_task
            is not None
            and not self._cleanup_task.done()
        ):
            return

        self._cleanup_task = (
            asyncio.create_task(
                self._cleanup_loop()
            )
        )

    async def stop(
        self,
    ):
        task = self._cleanup_task

        if task is None:
            return

        task.cancel()

        try:
            await task

        except asyncio.CancelledError:
            pass

        self._cleanup_task = None

    # ==================================================
    # TOKENS
    # ==================================================

    @staticmethod
    def _new_token() -> str:
        return secrets.token_urlsafe(
            32
        )

    # ==================================================
    # STATUS
    # ==================================================

    async def status(
        self,
        token: Optional[str],
    ) -> dict:
        async with self._lock:
            return self._status_locked(
                token,
                time.monotonic(),
            )

    def _status_locked(
        self,
        token: Optional[str],
        now: float,
    ) -> dict:
        personal_state = "none"
        queue_position = None
        grant_remaining = None
        lease_remaining = None

        if (
            token is not None
            and token
            == self._active_token
        ):
            personal_state = (
                "releasing"
                if self._releasing
                else "active"
            )

            if (
                self._active_started_at
                is not None
                and
                self._active_last_activity
                is not None
            ):
                hard_remaining = (
                    self.active_max_duration
                    - (
                        now
                        - self._active_started_at
                    )
                )

                idle_remaining = (
                    self.active_idle_timeout
                    - (
                        now
                        - self._active_last_activity
                    )
                )

                lease_remaining = max(
                    0,
                    int(
                        min(
                            hard_remaining,
                            idle_remaining,
                        )
                    ),
                )

        elif (
            token is not None
            and token
            == self._pending_token
        ):
            personal_state = "granted"

            if (
                self._pending_expires_at
                is not None
            ):
                grant_remaining = max(
                    0,
                    int(
                        self._pending_expires_at
                        - now
                    ),
                )

        elif token is not None:
            for (
                index,
                entry,
            ) in enumerate(
                self._queue,
                start=1,
            ):
                if (
                    entry.token
                    == token
                ):
                    personal_state = (
                        "queued"
                    )

                    queue_position = (
                        index
                    )

                    break

        if self._releasing:
            global_state = (
                "releasing"
            )

        elif (
            self._active_token
            is not None
            or self._pending_token
            is not None
        ):
            global_state = "in_use"

        else:
            global_state = (
                "available"
            )

        return {
            "global_state": (
                global_state
            ),
            "personal_state": (
                personal_state
            ),
            "queue_position": (
                queue_position
            ),
            "queue_length": len(
                self._queue
            ),
            "grant_remaining_seconds": (
                grant_remaining
            ),
            "lease_remaining_seconds": (
                lease_remaining
            ),
            "can_control": (
                personal_state
                == "active"
                and not self._releasing
            ),
        }

    # ==================================================
    # REQUEST / QUEUE
    # ==================================================

    async def request(
        self,
        token: Optional[str],
    ) -> dict:
        async with self._lock:
            now = time.monotonic()

            self._remove_stale_queued_locked(
                now
            )

            if not token:
                token = (
                    self._new_token()
                )

            if (
                token
                == self._active_token
                or token
                == self._pending_token
                or self._find_queue_entry_locked(
                    token
                )
                is not None
            ):
                return {
                    "session_token": token,
                    **self._status_locked(
                        token,
                        now,
                    ),
                }

            simulator_free = (
                self._active_token
                is None
                and self._pending_token
                is None
                and not self._releasing
                and not self._queue
            )

            if simulator_free:
                self._activate_locked(
                    token,
                    now,
                )

            else:
                if (
                    len(self._queue)
                    >= self.max_queue_size
                ):
                    raise QueueFullError(
                        "The simulator queue "
                        "is currently full."
                    )

                self._queue.append(
                    QueueEntry(
                        token=token,
                        joined_at=now,
                        last_seen=now,
                    )
                )

                if (
                    self._active_token
                    is None
                    and
                    self._pending_token
                    is None
                    and not self._releasing
                ):
                    self._promote_next_locked(
                        now
                    )

            self._notify_locked(
                now
            )

            return {
                "session_token": token,
                **self._status_locked(
                    token,
                    now,
                ),
            }

    # ==================================================
    # HEARTBEAT
    # ==================================================

    async def heartbeat(
        self,
        token: str,
    ) -> dict:
        async with self._lock:
            now = time.monotonic()

            if (
                token
                == self._active_token
            ):
                self._active_last_seen = (
                    now
                )

            entry = (
                self._find_queue_entry_locked(
                    token
                )
            )

            if entry is not None:
                entry.last_seen = now

            return self._status_locked(
                token,
                now,
            )

    # ==================================================
    # GRANT ACKNOWLEDGEMENT
    # ==================================================

    async def acknowledge(
        self,
        token: str,
    ) -> dict:
        async with self._lock:
            now = time.monotonic()

            if (
                token
                == self._active_token
                and not self._releasing
            ):
                return self._status_locked(
                    token,
                    now,
                )

            if (
                token
                != self._pending_token
            ):
                raise SessionStateError(
                    "This browser does not "
                    "currently hold a simulator grant."
                )

            if (
                self._pending_expires_at
                is not None
                and now
                > self._pending_expires_at
            ):
                self._pending_token = None
                self._pending_expires_at = (
                    None
                )

                self._promote_next_locked(
                    now
                )

                self._notify_locked(
                    now
                )

                raise SessionStateError(
                    "The simulator grant "
                    "has expired."
                )

            self._pending_token = None
            self._pending_expires_at = (
                None
            )

            self._activate_locked(
                token,
                now,
            )

            self._notify_locked(
                now
            )

            return self._status_locked(
                token,
                now,
            )

    # ==================================================
    # AUTHORIZATION
    # ==================================================

    async def require_active(
        self,
        token: Optional[str],
    ):
        if not token:
            raise SessionPermissionError(
                "An active simulator "
                "session is required."
            )

        async with self._lock:
            if (
                token
                != self._active_token
                or self._releasing
            ):
                raise SessionPermissionError(
                    "This browser does not "
                    "control the simulator."
                )

            now = time.monotonic()

            self._active_last_activity = (
                now
            )

            self._active_last_seen = now

    # ==================================================
    # RELEASE
    # ==================================================

    async def release(
        self,
        token: str,
    ) -> dict:
        async with self._lock:
            now = time.monotonic()

            entry = (
                self._find_queue_entry_locked(
                    token
                )
            )

            if entry is not None:
                self._queue.remove(
                    entry
                )

                self._notify_locked(
                    now
                )

                return self._status_locked(
                    token,
                    now,
                )

            if (
                token
                == self._pending_token
            ):
                self._pending_token = None
                self._pending_expires_at = (
                    None
                )

                self._promote_next_locked(
                    now
                )

                self._notify_locked(
                    now
                )

                return self._status_locked(
                    token,
                    now,
                )

            if (
                token
                != self._active_token
            ):
                return self._status_locked(
                    token,
                    now,
                )

        await self._release_active(
            token
        )

        return await self.status(
            token
        )

    async def _release_active(
        self,
        token: str,
    ):
        async with self._lock:
            if (
                token
                != self._active_token
                or self._releasing
            ):
                return

            self._releasing = True

            self._notify_locked(
                time.monotonic()
            )

        try:
            if (
                self._on_active_release
                is not None
            ):
                await (
                    self._on_active_release()
                )

        finally:
            async with self._lock:
                if (
                    token
                    == self._active_token
                ):
                    self._active_token = (
                        None
                    )

                    self._active_started_at = (
                        None
                    )

                    self._active_last_activity = (
                        None
                    )

                    self._active_last_seen = (
                        None
                    )

                self._releasing = False

                now = time.monotonic()

                self._promote_next_locked(
                    now
                )

                self._notify_locked(
                    now
                )

    # ==================================================
    # INTERNAL QUEUE HELPERS
    # ==================================================

    def _activate_locked(
        self,
        token: str,
        now: float,
    ):
        self._active_token = token
        self._active_started_at = (
            now
        )
        self._active_last_activity = (
            now
        )
        self._active_last_seen = (
            now
        )

    def _promote_next_locked(
        self,
        now: float,
    ):
        if (
            self._active_token
            is not None
            or self._pending_token
            is not None
            or self._releasing
        ):
            return

        self._remove_stale_queued_locked(
            now
        )

        if not self._queue:
            return

        entry = self._queue.popleft()

        self._pending_token = (
            entry.token
        )

        self._pending_expires_at = (
            now
            + self.grant_timeout
        )

    def _find_queue_entry_locked(
        self,
        token: str,
    ) -> Optional[QueueEntry]:
        for entry in self._queue:
            if entry.token == token:
                return entry

        return None

    def _remove_stale_queued_locked(
        self,
        now: float,
    ):
        self._queue = deque(
            entry
            for entry in self._queue
            if (
                now
                - entry.last_seen
                <= self.queue_heartbeat_timeout
            )
        )

    # ==================================================
    # SUBSCRIBERS
    # ==================================================

    async def subscribe(
        self,
        token: Optional[str],
    ) -> asyncio.Queue:
        async with self._lock:
            queue = asyncio.Queue(
                maxsize=20
            )

            self._subscribers.setdefault(
                token,
                set(),
            ).add(
                queue
            )

            queue.put_nowait(
                self._status_locked(
                    token,
                    time.monotonic(),
                )
            )

            return queue

    async def unsubscribe(
        self,
        token: Optional[str],
        queue: asyncio.Queue,
    ):
        async with self._lock:
            subscribers = (
                self._subscribers.get(
                    token
                )
            )

            if not subscribers:
                return

            subscribers.discard(
                queue
            )

            if not subscribers:
                self._subscribers.pop(
                    token,
                    None,
                )

    def _notify_locked(
        self,
        now: float,
    ):
        for (
            token,
            queues,
        ) in list(
            self._subscribers.items()
        ):
            status = (
                self._status_locked(
                    token,
                    now,
                )
            )

            for queue in list(
                queues
            ):
                if queue.full():
                    try:
                        queue.get_nowait()

                    except asyncio.QueueEmpty:
                        pass

                try:
                    queue.put_nowait(
                        status
                    )

                except asyncio.QueueFull:
                    pass

    # ==================================================
    # CLEANUP
    # ==================================================

    async def _cleanup_loop(
        self,
    ):
        while True:
            await asyncio.sleep(
                5
            )

            token_to_release = None

            async with self._lock:
                now = time.monotonic()

                previous_queue_length = (
                    len(self._queue)
                )

                self._remove_stale_queued_locked(
                    now
                )

                queue_changed = (
                    len(self._queue)
                    != previous_queue_length
                )

                if (
                    self._pending_token
                    is not None
                    and
                    self._pending_expires_at
                    is not None
                    and now
                    > self._pending_expires_at
                ):
                    self._pending_token = (
                        None
                    )

                    self._pending_expires_at = (
                        None
                    )

                    self._promote_next_locked(
                        now
                    )

                    queue_changed = True

                if (
                    self._active_token
                    is not None
                    and not self._releasing
                    and
                    self._active_started_at
                    is not None
                    and
                    self._active_last_activity
                    is not None
                ):
                    hard_expired = (
                        now
                        - self._active_started_at
                        >= self.active_max_duration
                    )

                    idle_expired = (
                        now
                        - self._active_last_activity
                        >= self.active_idle_timeout
                    )

                    if (
                        hard_expired
                        or idle_expired
                    ):
                        token_to_release = (
                            self._active_token
                        )

                if queue_changed:
                    self._notify_locked(
                        now
                    )

            if (
                token_to_release
                is not None
            ):
                await (
                    self._release_active(
                        token_to_release
                    )
                )