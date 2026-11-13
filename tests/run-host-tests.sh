#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d)
trap 'rm -rf "$test_dir"' EXIT
for suite in tests/*_test.cpp; do
    clang++ -std=c++17 -Wall -Wextra -Werror -pedantic \
        -fsanitize=undefined -fno-sanitize-recover=all \
        -Itests/fixture -IRobotFirmware/lib/HexaNRL/src -Ilib/HexaServos/src \
        "$suite" -o "$test_dir/test"
    "$test_dir/test"
done
