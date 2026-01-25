#!/usr/bin/env bash

set -e
set -o pipefail

clang-format -i $(find include -type f -name "*.hpp")
clang-format -i $(find src -type f -name "*.cpp")
