#!/usr/bin/env bash
# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# insert_package_copy.sh
#
# Add a COPY instruction for a given package to docker/robot/robot.Dockerfile
# right after the “# Copy in source code” comment. Script does nothing if the COPY line is already there.

set -euo pipefail

DOCKERFILE="docker/robot/robot.Dockerfile"
MARKER="# Copy in source code"

package_name="${1:-}"
if [[ -z "$package_name" ]]; then
    read -rp "Enter package_name: " package_name
fi
[[ -z "$package_name" ]] && { echo "Error: package_name cannot be empty." >&2; exit 1; }

copy_line="COPY src/robot/${package_name} ${package_name}"

[[ -f "$DOCKERFILE" ]] || { echo "Error: $DOCKERFILE not found." >&2; exit 1; }
if grep -Fxq "$copy_line" "$DOCKERFILE"; then
    echo "COPY line already present – no changes made."
    exit 0
fi
grep -Fq "$MARKER" "$DOCKERFILE" || { echo "Error: marker '$MARKER' not found." >&2; exit 1; }

# GNU sed and BSD sed use different -i semantics, so handle both.
if sed --version >/dev/null 2>&1; then
    # GNU sed
    sed -i "/$MARKER/a $copy_line" "$DOCKERFILE"
else
    # BSD sed (macOS, *BSD)
    sed -i '' "/$MARKER/a\\
$copy_line
" "$DOCKERFILE"
fi

echo "Inserted COPY line for '$package_name' into $DOCKERFILE"
