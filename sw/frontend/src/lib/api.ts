function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === "object" && value !== null;
}

/**
 * Build an SD-card file endpoint without allowing file-name characters to
 * alter the URL path structure. Path separators remain literal because the
 * firmware uses them to identify nested directories.
 */
export function sdCardFileEndpoint(path: string, isDirectory = false): string {
  let sdPath = path.startsWith("/") ? path : `/${path}`;

  if (isDirectory && !sdPath.endsWith("/")) {
    sdPath += "/";
  }

  const encodedPath = sdPath
    .split("/")
    .map((segment) => encodeURIComponent(segment))
    .join("/");

  return `/api/v1/sd_card/file${encodedPath}`;
}

function responseError(response: Response, payload: unknown): Error {
  const reason = isRecord(payload) && payload.reason != null
    ? String(payload.reason).trim()
    : "";

  if (reason) {
    return new Error(reason);
  }

  if (!response.ok) {
    const status = response.statusText
      ? `${response.status} ${response.statusText}`
      : String(response.status);
    return new Error(`HTTP ${status}`);
  }

  return new Error("Request failed");
}

/** Parse an API JSON response and reject both HTTP and firmware-level errors. */
export async function parseJsonResponse<T>(response: Response): Promise<T> {
  let payload: unknown;

  try {
    payload = await response.json();
  } catch {
    if (!response.ok) {
      throw responseError(response, null);
    }
    throw new Error("Invalid JSON response");
  }

  if (!response.ok || (isRecord(payload) && payload.status === "error")) {
    throw responseError(response, payload);
  }

  return payload as T;
}

/**
 * Parse an API response whose successful endpoint may legitimately have no body.
 *
 * This is intentionally separate from parseJsonResponse: callers that require a
 * JSON document should continue to reject an empty successful response.
 */
export async function parseJsonResponseAllowEmpty<T>(response: Response): Promise<T | undefined> {
  let body: string;

  try {
    body = await response.text();
  } catch {
    if (!response.ok) {
      throw responseError(response, null);
    }
    throw new Error("Invalid JSON response");
  }

  if (body.trim() === "") {
    if (!response.ok) {
      throw responseError(response, null);
    }
    return undefined;
  }

  let payload: unknown;
  try {
    payload = JSON.parse(body);
  } catch {
    if (!response.ok) {
      throw responseError(response, null);
    }
    throw new Error("Invalid JSON response");
  }

  if (!response.ok || (isRecord(payload) && payload.status === "error")) {
    throw responseError(response, payload);
  }

  return payload as T;
}
