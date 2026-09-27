from .models import UAVState


ALLOWED_TRANSITIONS = {
    UAVState.DISCONNECTED: {
        UAVState.INITIALIZING,
        UAVState.FAILED,
    },

    UAVState.INITIALIZING: {
        UAVState.READY,
        UAVState.FAILED,
    },

    UAVState.READY: {
        UAVState.ARMING,
        UAVState.FAILED,
    },

    UAVState.ARMING: {
        UAVState.TAKING_OFF,
        UAVState.FAILED,
    },

    UAVState.TAKING_OFF: {
        UAVState.AIRBORNE,
        UAVState.FAILED,
    },

    UAVState.AIRBORNE: {
        UAVState.SEARCHING,
        UAVState.RECOVERING,
        UAVState.RETURNING,
        UAVState.LANDING,
        UAVState.FAILED,
    },

    UAVState.SEARCHING: {
        UAVState.AIRBORNE,
        UAVState.RECOVERING,
        UAVState.RETURNING,
        UAVState.LANDING,
        UAVState.FAILED,
    },

    UAVState.RECOVERING: {
        UAVState.AIRBORNE,
        UAVState.SEARCHING,
        UAVState.RETURNING,
        UAVState.LANDING,
        UAVState.FAILED,
    },

    UAVState.RETURNING: {
        UAVState.LANDING,
        UAVState.FAILED,
    },

    UAVState.LANDING: {
        UAVState.LANDED,
        UAVState.FAILED,
    },

    UAVState.LANDED: set(),

    # FAILED is terminal for this mission.
    UAVState.FAILED: set(),
}


def validate_transition(
    current: UAVState,
    new: UAVState,
) -> None:
    if current == new:
        return

    if new not in ALLOWED_TRANSITIONS[current]:
        raise RuntimeError(
            f"Invalid UAV state transition: "
            f"{current.value} -> {new.value}"
        )