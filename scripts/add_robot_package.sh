#!/usr/bin/env bash
set -euo pipefail

DOCKERFILE="docker/robot/robot.Dockerfile"
LAUNCH_FILE="src/robot/bringup_robot/launch/robot.launch.py"
BR_PKG_XML="src/robot/bringup_robot/package.xml"
TEMPLATE_DIR="templates"

pkg="${1:-}"
[[ -z "$pkg" ]] && { echo "Usage: $0 <package_name>" >&2; exit 1; }
[[ "$pkg" =~ [^a-zA-Z0-9_]+ ]] && { echo "Error: package name must be alnum/underscore." >&2; exit 1; }

scripts/insert_package_copy.sh "$pkg"

CamelPkg="$(tr '[:lower:]' '[:upper:]' <<< "${pkg:0:1}")${pkg:1}"
block=$(cat <<EOF
#################### ${CamelPkg} Node #####################
    ${pkg}_node = Node(
        package='${pkg}',
        name='${pkg}_node',
        executable='${pkg}_node',
    )
    ld.add_action(${pkg}_node)

EOF
)

if ! grep -Fq "${pkg}_node = Node(" "$LAUNCH_FILE"; then
    tmp=$(mktemp)
    block_file=$(mktemp)
    printf '%s' "$block" > "$block_file"
    awk -v bf="$block_file" '
        $0 ~ /##[[:space:]]+LAUNCH[[:space:]]+NODES/ && !done {
            print
            while ((getline line < bf) > 0) print line
            close(bf); done=1; next
        }
        { print }
    ' "$LAUNCH_FILE" > "$tmp" && mv "$tmp" "$LAUNCH_FILE"
    rm -f "$block_file"
    echo "✓ Added node block to '$LAUNCH_FILE'"
else
    echo "• Launch-file block already present – skipped"
fi

dep_line="  <depend>${pkg}</depend>"
if ! grep -Fq "$dep_line" "$BR_PKG_XML"; then
    if sed --version >/dev/null 2>&1; then
        sed -i "/<!-- DEPENDENCIES -->/a ${dep_line}" "$BR_PKG_XML"
    else
        sed -i '' "/<!-- DEPENDENCIES -->/a\\
${dep_line}
" "$BR_PKG_XML"
    fi
    echo "✓ Added dependency to '$BR_PKG_XML'"
else
    echo "• Dependency already present – skipped"
fi

dest_root="src/robot/${pkg}"
if [[ -d "$dest_root" ]]; then
    echo "• Directory '$dest_root' already exists – no files generated"
    exit 0
fi

echo "Creating new package directory tree at '$dest_root'"
while IFS= read -r -d '' templ; do
    rel="${templ#$TEMPLATE_DIR/}"
    [[ $rel == "params.yaml.in" ]] && rel="config/$rel"
    rel="${rel//package_name/${pkg}}"
    rel="${rel%.in}"
    dest="$dest_root/$rel"
    mkdir -p "$(dirname "$dest")"
    sed "s/package_name/${pkg}/g; s/PackageName/${CamelPkg}/g" "$templ" > "$dest"
done < <(find "$TEMPLATE_DIR" -type f -print0)

echo "Done, templates expanded"
echo -e "\nAll done – build & run with ./watod build and ./watod up as usual"
