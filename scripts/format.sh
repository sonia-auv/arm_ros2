#!/usr/bin/env bash

set -e
set -o pipefail

clang-format -i $(find packages -type f -name "*.hpp")
clang-format -i $(find packages -type f -name "*.cpp")
