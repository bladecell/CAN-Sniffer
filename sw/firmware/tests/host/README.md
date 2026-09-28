# Host regressions

Run the dependency-free PID priority queue regressions from `sw/firmware` or
anywhere else in the repository:

```sh
bash sw/firmware/tests/host/run_pid_priority_queue_test.sh
```

The runner compiles the production queue header against the small FreeRTOS API
fakes in `fakes/`. It writes the executable under `/tmp/opencode`, not into the
repository. These tests cover queue behavior only; they do not simulate FreeRTOS
task scheduling or replace the on-target lifecycle tests.
