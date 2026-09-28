import {
  useEffect,
  useMemo,
  useState,
} from "react";

import "./App.css";

import MissionScene from "./components/MissionScene";

import {
  useMissionControl,
} from "./useMissionControl";


function titleCase(
  value,
) {
  if (!value) {
    return "Unknown";
  }

  return value
    .replaceAll(
      "_",
      " ",
    )
    .replace(
      /\b\w/g,
      (letter) =>
        letter.toUpperCase(),
    );
}


function Status({
  label,
  value,
  good = false,
}) {
  return (
    <div className="status-readout">
      <span>
        {label}
      </span>

      <strong
        className={
          good
            ? "good"
            : ""
        }
      >
        {value}
      </strong>
    </div>
  );
}


function SimulatorControl({
  session,
  busy,
  onClaim,
  onEnter,
  onLeave,
}) {
  const personal =
    session.personal_state;

  if (
    personal === "active"
  ) {
    return (
      <div className="simulator-control">
        <span className="simulator-state active">
          Simulator yours
        </span>

        <button
          onClick={onLeave}
          disabled={busy}
        >
          Release
        </button>
      </div>
    );
  }

  if (
    personal === "releasing"
  ) {
    return (
      <div className="simulator-control">
        <span className="simulator-state">
          Releasing simulator
        </span>
      </div>
    );
  }

  if (
    personal === "granted"
  ) {
    return (
      <div className="simulator-control">
        <span className="simulator-state ready">
          Your turn
        </span>

        <button
          onClick={onEnter}
          disabled={busy}
        >
          Enter
        </button>
      </div>
    );
  }

  if (
    personal === "queued"
  ) {
    return (
      <div className="simulator-control">
        <span className="simulator-state">
          Queue #
          {
            session.queue_position
          }
        </span>

        <button
          onClick={onLeave}
          disabled={busy}
        >
          Leave
        </button>
      </div>
    );
  }

  const available =
    session.global_state ===
    "available";

  return (
    <div className="simulator-control">
      <span
        className={
          available
            ? "simulator-state ready"
            : "simulator-state"
        }
      >
        {available
          ? "Simulator available"
          : `Simulator in use · ${session.queue_length ?? 0} waiting`}
      </span>

      <button
        onClick={onClaim}
        disabled={busy}
      >
        {available
          ? "Launch"
          : "Join queue"}
      </button>
    </div>
  );
}


function MissionForm({
  config,
  disabled,
  busy,
  onSubmit,
}) {
  const [
    form,
    setForm,
  ] = useState({
    width_m:
      config?.width_m ??
      60,

    height_m:
      config?.height_m ??
      30,

    lane_spacing_m:
      config?.lane_spacing_m ??
      15,
  });

  useEffect(() => {
    if (
      !disabled &&
      config
    ) {
      setForm({
        width_m:
          config.width_m ??
          60,

        height_m:
          config.height_m ??
          30,

        lane_spacing_m:
          config
            .lane_spacing_m ??
          15,
      });
    }
  }, [
    config,
    disabled,
  ]);

  function update(
    key,
    value,
  ) {
    setForm(
      (previous) => ({
        ...previous,
        [key]:
          Number(value),
      }),
    );
  }

  function submit(
    event,
  ) {
    event.preventDefault();

    onSubmit(form);
  }

  return (
    <form
      className="mission-controls"
      onSubmit={submit}
    >
      <div className="panel-title">
        Mission
      </div>

      <div className="mission-fields">
        <label>
          <span>
            Width
          </span>

          <div className="input-unit">
            <input
              type="number"
              min="1"
              max="1000"
              value={
                form.width_m
              }
              disabled={
                disabled
              }
              onChange={(
                event,
              ) =>
                update(
                  "width_m",
                  event
                    .target
                    .value,
                )
              }
            />

            <i>
              m
            </i>
          </div>
        </label>

        <label>
          <span>
            Height
          </span>

          <div className="input-unit">
            <input
              type="number"
              min="1"
              max="1000"
              value={
                form.height_m
              }
              disabled={
                disabled
              }
              onChange={(
                event,
              ) =>
                update(
                  "height_m",
                  event
                    .target
                    .value,
                )
              }
            />

            <i>
              m
            </i>
          </div>
        </label>

        <label>
          <span>
            Lane
          </span>

          <div className="input-unit">
            <input
              type="number"
              min="1"
              max="250"
              value={
                form
                  .lane_spacing_m
              }
              disabled={
                disabled
              }
              onChange={(
                event,
              ) =>
                update(
                  "lane_spacing_m",
                  event
                    .target
                    .value,
                )
              }
            />

            <i>
              m
            </i>
          </div>
        </label>
      </div>

      <button
        className="start-button"
        type="submit"
        disabled={
          disabled ||
          busy
        }
      >
        {busy
          ? "Starting"
          : disabled
            ? "Mission locked"
            : "Start mission"}
      </button>
    </form>
  );
}


function Timeline({
  events,
}) {
  const visible =
    useMemo(
      () =>
        [
          ...(events ?? []),
        ]
          .slice(-12)
          .reverse(),
      [events],
    );

  return (
    <div className="timeline-panel">
      <div className="panel-title">
        Events
      </div>

      <div className="timeline-list">
        {visible.length ===
        0 ? (
          <div className="quiet-text">
            Waiting for mission events.
          </div>
        ) : (
          visible.map(
            (
              event,
              index,
            ) => (
              <div
                className="event-row"
                key={
                  `${event.timestamp}-${index}`
                }
              >
                <time>
                  {new Date(
                    event.timestamp,
                  ).toLocaleTimeString(
                    [],
                    {
                      hour:
                        "2-digit",
                      minute:
                        "2-digit",
                      second:
                        "2-digit",
                    },
                  )}
                </time>

                <span className="event-source">
                  {
                    event.source
                  }
                </span>

                <span>
                  {
                    event.message
                  }
                </span>
              </div>
            ),
          )
        )}
      </div>
    </div>
  );
}


function VehicleCard({
  name,
  uav,
  missionRunning,
  onFailure,
}) {
  const failed =
    uav.healthy === false ||
    uav.state === "failed";

  const canFail =
    missionRunning &&
    !failed &&
    uav.state !== "landed";

  return (
    <article
      className={
        failed
          ? "vehicle-row failed"
          : "vehicle-row"
      }
    >
      <div className="vehicle-top">
        <div>
          <strong>
            {name}
          </strong>

          <span>
            {titleCase(
              uav.state,
            )}
          </span>
        </div>

        <div
          className={
            failed
              ? "health failed"
              : uav.armed
                ? "health armed"
                : "health safe"
          }
        >
          {failed
            ? "FAILED"
            : uav.armed
              ? "ARMED"
              : "SAFE"}
        </div>
      </div>

      <div className="vehicle-data">
        <div>
          <span>
            ALT
          </span>

          <strong>
            {(
              uav.altitude ??
              0
            ).toFixed(1)}
            m
          </strong>
        </div>

        <div>
          <span>
            MODE
          </span>

          <strong>
            {
              uav.flight_mode ??
              "—"
            }
          </strong>
        </div>

        <div>
          <span>
            E
          </span>

          <strong>
            {uav.east == null
              ? "—"
              : `${uav.east.toFixed(
                  1,
                )}m`}
          </strong>
        </div>

        <div>
          <span>
            N
          </span>

          <strong>
            {uav.north == null
              ? "—"
              : `${uav.north.toFixed(
                  1,
                )}m`}
          </strong>
        </div>
      </div>

      {uav.failure_reason && (
        <div className="failure-text">
          {
            uav.failure_reason
          }
        </div>
      )}

      <button
        className="failure-button"
        disabled={!canFail}
        onClick={() =>
          onFailure(name)
        }
      >
        Inject failure
      </button>
    </article>
  );
}


export default function App() {
  const {
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
  } = useMissionControl();

  const runtime =
    state.runtime ?? {};

  const world =
    state.world ?? {};

  const mission =
    world.mission ?? {};

  const uavs =
    world.uavs ?? {};

  const missionState =
    mission.state ??
    "idle";

  const ownsSimulator =
    session.personal_state ===
    "active";

  const canStartMission =
    ownsSimulator &&
    (
      runtime
        .accepts_new_mission ??
      false
    );

  return (
    <div className="app">
      <header className="utility-bar">
        <SimulatorControl
          session={session}
          busy={busy}
          onClaim={
            claimSimulator
          }
          onEnter={
            enterSimulator
          }
          onLeave={
            leaveSimulator
          }
        />

        <div className="utility-status">
          <Status
            label="Mission"
            value={titleCase(
              missionState,
            )}
            good={
              missionState ===
                "idle" ||
              missionState ===
                "completed"
            }
          />

          <Status
            label="Socket"
            value={titleCase(
              socketStatus,
            )}
            good={
              socketStatus ===
              "connected"
            }
          />
        </div>
      </header>

      {error && (
        <div className="error-banner">
          {error}
        </div>
      )}

      <main className="workspace">
        <section className="scene-panel">
          <MissionScene
            config={
              state
                .mission_config
            }
            plan={
              state.plan
            }
            uavs={uavs}
          />

          <div className="controls-overlay">
            <MissionForm
              config={
                state
                  .mission_config
              }
              disabled={
                !canStartMission
              }
              busy={busy}
              onSubmit={
                startMission
              }
            />
          </div>

          <div className="summary-overlay">
            <div>
              <span>
                RECOVERY
              </span>

              <strong>
                {mission
                  .recovery_count ??
                  0}
              </strong>
            </div>

            <div>
              <span>
                FAILED
              </span>

              <strong>
                {mission
                  .failed_uavs
                  ?.length ??
                  0}
              </strong>
            </div>
          </div>

          <div className="timeline-overlay">
            <Timeline
              events={
                world.timeline
              }
            />
          </div>
        </section>

        <aside className="fleet-rail">
          <div className="fleet-heading">
            <span>
              Fleet
            </span>

            <strong>
              {
                Object.keys(
                  uavs,
                ).length
              }
            </strong>
          </div>

          <div className="fleet-list">
            {Object.keys(
              uavs,
            ).length ===
            0 ? (
              <div className="quiet-text fleet-empty">
                Start a mission to
                connect aircraft.
              </div>
            ) : (
              Object.entries(
                uavs,
              ).map(
                ([
                  name,
                  uav,
                ]) => (
                  <VehicleCard
                    key={name}
                    name={name}
                    uav={uav}
                    missionRunning={
                      ownsSimulator &&
                      runtime
                        .mission_running
                    }
                    onFailure={
                      injectFailure
                    }
                  />
                ),
              )
            )}
          </div>
        </aside>
      </main>
    </div>
  );
}