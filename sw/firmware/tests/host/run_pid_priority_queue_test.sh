#!/usr/bin/env bash
set -euo pipefail
root="$(cd "$(dirname "$0")/../.." && pwd)"
build_dir="/tmp/opencode/pid-priority-queue-host"
mkdir -p "$build_dir"
c++ -std=c++17 -Wall -Wextra -Werror -Wno-non-c-typedef-for-linkage \
  -I"$root/tests/host/fakes" \
  -I"$root/components/obd2/include" \
  "$root/tests/host/pid_priority_queue_test.cpp" \
  -o "$build_dir/pid_priority_queue_test"
"$build_dir/pid_priority_queue_test"
