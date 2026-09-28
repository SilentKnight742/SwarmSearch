import asyncio

from backend.session_manager import (
    SessionManager,
)


def test_fifo_queue_and_grant():
    async def run():
        manager = (
            SessionManager()
        )

        first = (
            await manager.request(
                None
            )
        )

        first_token = (
            first["session_token"]
        )

        assert (
            first[
                "personal_state"
            ]
            == "active"
        )

        second = (
            await manager.request(
                None
            )
        )

        second_token = (
            second["session_token"]
        )

        assert (
            second[
                "personal_state"
            ]
            == "queued"
        )

        assert (
            second[
                "queue_position"
            ]
            == 1
        )

        await manager.release(
            first_token
        )

        second_status = (
            await manager.status(
                second_token
            )
        )

        assert (
            second_status[
                "personal_state"
            ]
            == "granted"
        )

        await manager.acknowledge(
            second_token
        )

        second_status = (
            await manager.status(
                second_token
            )
        )

        assert (
            second_status[
                "personal_state"
            ]
            == "active"
        )

    asyncio.run(
        run()
    )


def test_queue_is_fifo():
    async def run():
        manager = (
            SessionManager()
        )

        first = (
            await manager.request(
                None
            )
        )

        second = (
            await manager.request(
                None
            )
        )

        third = (
            await manager.request(
                None
            )
        )

        await manager.release(
            first[
                "session_token"
            ]
        )

        second_status = (
            await manager.status(
                second[
                    "session_token"
                ]
            )
        )

        third_status = (
            await manager.status(
                third[
                    "session_token"
                ]
            )
        )

        assert (
            second_status[
                "personal_state"
            ]
            == "granted"
        )

        assert (
            third_status[
                "queue_position"
            ]
            == 1
        )

    asyncio.run(
        run()
    )