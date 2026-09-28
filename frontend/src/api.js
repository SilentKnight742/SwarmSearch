const API_BASE = "";

export const WS_BASE =
  `${window.location.protocol === "https:" ? "wss" : "ws"}://${window.location.host}/ws`;

const SESSION_HEADER =
  "X-SwarmSearch-Session";


async function request(
  path,
  {
    sessionToken = null,
    headers = {},
    ...options
  } = {},
) {
  const finalHeaders = {
    "Content-Type": "application/json",
    ...headers,
  };

  if (sessionToken) {
    finalHeaders[
      SESSION_HEADER
    ] = sessionToken;
  }

  const response =
    await fetch(
      `${API_BASE}${path}`,
      {
        ...options,
        headers: finalHeaders,
      },
    );

  const body =
    await response
      .json()
      .catch(() => ({}));

  if (!response.ok) {
    throw new Error(
      body.detail ??
        `Request failed with HTTP ${response.status}`,
    );
  }

  return body;
}


export function getMissionState() {
  return request(
    "/api/state",
  );
}


export function getSessionStatus(
  sessionToken,
) {
  return request(
    "/api/session",
    {
      sessionToken,
    },
  );
}


export function requestSimulator(
  sessionToken,
) {
  return request(
    "/api/session/request",
    {
      method: "POST",
      sessionToken,
    },
  );
}


export function heartbeatSimulator(
  sessionToken,
) {
  return request(
    "/api/session/heartbeat",
    {
      method: "POST",
      sessionToken,
    },
  );
}


export function acknowledgeSimulator(
  sessionToken,
) {
  return request(
    "/api/session/ack",
    {
      method: "POST",
      sessionToken,
    },
  );
}


export function releaseSimulator(
  sessionToken,
) {
  return request(
    "/api/session/release",
    {
      method: "POST",
      sessionToken,
    },
  );
}


export function startMission(
  config,
  sessionToken,
) {
  return request(
    "/api/mission/start",
    {
      method: "POST",
      sessionToken,
      body:
        JSON.stringify(
          config,
        ),
    },
  );
}


export function injectFailure(
  vehicleName,
  sessionToken,
  reason = (
    "frontend operator failure"
  ),
) {
  return request(
    `/api/mission/failure/${encodeURIComponent(
      vehicleName,
    )}`,
    {
      method: "POST",
      sessionToken,
      body:
        JSON.stringify({
          reason,
        }),
    },
  );
}