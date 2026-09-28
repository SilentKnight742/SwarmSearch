import {
  useCallback,
  useEffect,
  useState,
} from "react";

import {
  acknowledgeSimulator,
  getMissionState,
  getSessionStatus,
  heartbeatSimulator,
  injectFailure as injectFailureRequest,
  releaseSimulator,
  requestSimulator,
  startMission as startMissionRequest,
  WS_BASE,
} from "./api";


const EMPTY_STATE = {
  runtime: {
    mission_running: false,
    fleet_armed: false,
    accepts_new_mission: true,
  },

  mission_config: {
    width_m: 60,
    height_m: 30,
    lane_spacing_m: 15,
  },

  plan: {},

  world: {
    mission: {
      state: "idle",
      failed_uavs: [],
      recovery_count: 0,
    },

    origin: null,
    uavs: {},
    timeline: [],
  },
};


const EMPTY_SESSION = {
  global_state: "available",
  personal_state: "none",
  queue_position: null,
  queue_length: 0,
  grant_remaining_seconds: null,
  lease_remaining_seconds: null,
  can_control: false,
};


function ensureUav(
  uavs,
  name,
) {
  return (
    uavs[name] ?? {
      name,
      state: "disconnected",
      latitude: null,
      longitude: null,
      east: null,
      north: null,
      altitude: 0,
      target_altitude: null,
      armed: false,
      flight_mode: null,
      healthy: true,
      failure_reason: null,
    }
  );
}


function applyEvent(
  previous,
  message,
) {
  const event =
    message.data;

  const eventType =
    event.event_type;

  const source =
    event.source;

  const data =
    event.data ?? {};

  const world = {
    ...previous.world,

    mission: {
      ...previous.world.mission,
    },

    uavs: {
      ...previous.world.uavs,
    },

    timeline: [
      ...(previous.world.timeline ??
        []),
    ],
  };

  if (
    eventType ===
    "mission.state_changed"
  ) {
    world.mission.state =
      data.state;
  }

  if (
    eventType ===
    "mission.origin_ready"
  ) {
    world.origin = {
      latitude:
        data.latitude,
      longitude:
        data.longitude,
    };
  }

  if (
    eventType ===
    "mission.route_reassigned"
  ) {
    world.mission
      .recovery_count =
      (
        world.mission
          .recovery_count ??
        0
      ) + 1;
  }

  if (
    eventType ===
    "mission.completed"
  ) {
    world.mission.failed_uavs =
      data.failed_uavs ??
      world.mission.failed_uavs;

    world.mission.recovery_count =
      data.recovery_count ??
      world.mission
        .recovery_count;
  }

  if (
    source.startsWith(
      "UAV-",
    )
  ) {
    const uav = {
      ...ensureUav(
        world.uavs,
        source,
      ),
    };

    if (
      eventType ===
      "uav.state_changed"
    ) {
      uav.state =
        data.state;
    }

    if (
      eventType ===
      "uav.takeoff_started"
    ) {
      uav.target_altitude =
        data.target_altitude;
    }

    if (
      eventType ===
      "uav.telemetry"
    ) {
      uav.latitude =
        data.latitude;

      uav.longitude =
        data.longitude;

      uav.east =
        data.east;

      uav.north =
        data.north;

      uav.altitude =
        data.altitude ??
        uav.altitude;

      uav.armed =
        data.armed ??
        uav.armed;

      uav.flight_mode =
        data.flight_mode ??
        uav.flight_mode;

      uav.healthy =
        data.healthy ??
        uav.healthy;

      if (
        data.mission_state
      ) {
        uav.state =
          data.mission_state;
      }
    }

    if (
      eventType ===
      "uav.failed"
    ) {
      uav.state = "failed";
      uav.healthy = false;

      uav.failure_reason =
        event.message;

      const failed =
        new Set(
          world.mission
            .failed_uavs ??
            [],
        );

      failed.add(source);

      world.mission
        .failed_uavs =
        [...failed];
    }

    world.uavs[
      source
    ] = uav;
  }

  if (
    eventType !==
      "uav.telemetry" &&
    eventType !==
      "uav.autopilot_text"
  ) {
    world.timeline.push(
      event
    );

    if (
      world.timeline.length
      > 200
    ) {
      world.timeline =
        world.timeline.slice(
          -200,
        );
    }
  }

  return {
    ...previous,

    runtime:
      message.runtime ??
      previous.runtime,

    world,
  };
}


export function useMissionControl() {
  const [
    state,
    setState,
  ] = useState(
    EMPTY_STATE,
  );

  const [
    session,
    setSession,
  ] = useState(
    EMPTY_SESSION,
  );

  const [
    sessionToken,
    setSessionToken,
  ] = useState(
    () =>
      sessionStorage.getItem(
        "swarmsearch_session_token",
      ),
  );

  const [
    socketStatus,
    setSocketStatus,
  ] = useState(
    "connecting",
  );

  const [
    error,
    setError,
  ] = useState(
    null,
  );

  const [
    busy,
    setBusy,
  ] = useState(
    false,
  );


  const refreshState =
    useCallback(
      async () => {
        try {
          const latest =
            await getMissionState();

          setState(latest);
        } catch (err) {
          setError(
            err.message,
          );
        }
      },
      [],
    );


  const refreshSession =
    useCallback(
      async () => {
        try {
          const latest =
            await getSessionStatus(
              sessionToken,
            );

          setSession(
            latest,
          );
        } catch (err) {
          setError(
            err.message,
          );
        }
      },
      [sessionToken],
    );


  useEffect(() => {
    refreshState();
    refreshSession();
  }, [
    refreshState,
    refreshSession,
  ]);


  useEffect(() => {
    if (!sessionToken) {
      return undefined;
    }

    const interval =
      setInterval(
        async () => {
          try {
            const latest =
              await heartbeatSimulator(
                sessionToken,
              );

            setSession(
              latest,
            );
          } catch {
            // WebSocket/session refresh
            // will recover status.
          }
        },
        20_000,
      );

    return () =>
      clearInterval(
        interval,
      );
  }, [
    sessionToken,
  ]);


  useEffect(() => {
    let cancelled = false;
    let socket = null;
    let reconnectTimer = null;

    const connect = () => {
      if (cancelled) {
        return;
      }

      setSocketStatus(
        "connecting",
      );

      const query =
        sessionToken
          ? `?session_token=${encodeURIComponent(
              sessionToken,
            )}`
          : "";

      socket =
        new WebSocket(
          `${WS_BASE}${query}`,
        );

      socket.onopen =
        () => {
          if (!cancelled) {
            setSocketStatus(
              "connected",
            );
          }
        };

      socket.onmessage =
        (
          websocketEvent,
        ) => {
          const message =
            JSON.parse(
              websocketEvent.data,
            );

          if (
            message.type ===
            "snapshot"
          ) {
            setState(
              message.data,
            );

            if (
              message.session
            ) {
              setSession(
                message.session,
              );
            }

            return;
          }

          if (
            message.type ===
            "session"
          ) {
            setSession(
              message.data,
            );

            return;
          }

          if (
            message.type ===
            "event"
          ) {
            setState(
              (previous) =>
                applyEvent(
                  previous,
                  message,
                ),
            );

            if (
              [
                "mission.origin_ready",
                "mission.route_assigned",
                "mission.route_reassigned",
                "mission.completed",
                "mission.aborted",
              ].includes(
                message.data
                  ?.event_type,
              )
            ) {
              setTimeout(
                refreshState,
                120,
              );
            }
          }
        };

      socket.onerror =
        () => {
          if (!cancelled) {
            setSocketStatus(
              "error",
            );
          }
        };

      socket.onclose =
        () => {
          if (cancelled) {
            return;
          }

          setSocketStatus(
            "disconnected",
          );

          reconnectTimer =
            setTimeout(
              connect,
              2000,
            );
        };
    };

    connect();

    return () => {
      cancelled = true;

      if (
        reconnectTimer
      ) {
        clearTimeout(
          reconnectTimer,
        );
      }

      if (socket) {
        socket.close();
      }
    };
  }, [
    sessionToken,
    refreshState,
  ]);


  const claimSimulator =
    useCallback(
      async () => {
        setBusy(true);
        setError(null);

        try {
          const result =
            await requestSimulator(
              sessionToken,
            );

          const token =
            result
              .session_token;

          if (
            token &&
            token !==
              sessionToken
          ) {
            sessionStorage.setItem(
              "swarmsearch_session_token",
              token,
            );

            setSessionToken(
              token,
            );
          }

          setSession(
            result,
          );
        } catch (err) {
          setError(
            err.message,
          );
        } finally {
          setBusy(false);
        }
      },
      [sessionToken],
    );


  const enterSimulator =
    useCallback(
      async () => {
        if (!sessionToken) {
          return;
        }

        setBusy(true);
        setError(null);

        try {
          const result =
            await acknowledgeSimulator(
              sessionToken,
            );

          setSession(
            result,
          );
        } catch (err) {
          setError(
            err.message,
          );
        } finally {
          setBusy(false);
        }
      },
      [sessionToken],
    );


  const leaveSimulator =
    useCallback(
      async () => {
        if (!sessionToken) {
          return;
        }

        setBusy(true);
        setError(null);

        try {
          const result =
            await releaseSimulator(
              sessionToken,
            );

          setSession(
            result,
          );

          sessionStorage.removeItem(
            "swarmsearch_session_token",
          );

          setSessionToken(
            null,
          );
        } catch (err) {
          setError(
            err.message,
          );
        } finally {
          setBusy(false);
        }
      },
      [sessionToken],
    );


  const startMission =
    useCallback(
      async (config) => {
        setBusy(true);
        setError(null);

        try {
          await startMissionRequest(
            config,
            sessionToken,
          );

          await refreshState();

          return true;
        } catch (err) {
          setError(
            err.message,
          );

          return false;
        } finally {
          setBusy(false);
        }
      },
      [
        refreshState,
        sessionToken,
      ],
    );


  const injectFailure =
    useCallback(
      async (
        vehicleName,
      ) => {
        setError(null);

        try {
          await injectFailureRequest(
            vehicleName,
            sessionToken,
          );

          return true;
        } catch (err) {
          setError(
            err.message,
          );

          return false;
        }
      },
      [sessionToken],
    );


  return {
    state,
    session,
    socketStatus,
    error,
    busy,

    claimSimulator,
    enterSimulator,
    leaveSimulator,

    startMission,
    injectFailure,
  };
}