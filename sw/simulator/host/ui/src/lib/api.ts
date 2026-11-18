export function apiHeaders(method: string, hasBody: boolean): HeadersInit {
  return {
    ...(hasBody ? { 'Content-Type': 'application/json' } : {}),
    ...(method === 'GET' ? {} : { 'X-Simulator-Intent': 'ui' })
  };
}
