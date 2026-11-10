/** Runs the tasks with at most `limit` in flight; a failing task does not stop the rest. */
export async function runLimited(tasks: Array<() => Promise<unknown>>, limit: number): Promise<void> {
  const queue = [...tasks];
  const worker = async () => {
    for (let task = queue.shift(); task; task = queue.shift()) await task().catch(() => undefined);
  };
  await Promise.all(Array.from({ length: Math.min(limit, queue.length) }, worker));
}
