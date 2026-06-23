#!/bin/sh
set -eu

repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
out_dir="${TMPDIR:-/tmp}/esp32s3_adapter_tests"
mkdir -p "$out_dir"

cc -std=c11 -Wall -Wextra -Werror \
  -I "$repo_root/main" \
  "$repo_root/test/host_app_protocol_test.c" \
  "$repo_root/main/app_protocol.c" \
  -o "$out_dir/host_app_protocol_test"

"$out_dir/host_app_protocol_test"
