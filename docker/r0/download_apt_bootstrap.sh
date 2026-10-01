#!/usr/bin/env bash
# Download, but never install, the apt closure needed to add pip to the
# digest-pinned base. Run only in an ephemeral networked bootstrap container.
set -euo pipefail

if [[ $# -ne 1 ]]; then
    echo "usage: $0 OUTPUT_DIRECTORY" >&2
    exit 64
fi

output_dir="$1"
if [[ ! -d "$output_dir" ]]; then
    echo "output directory does not exist: $output_dir" >&2
    exit 65
fi
if find "$output_dir" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
    echo "output directory is not empty: $output_dir" >&2
    exit 66
fi

apt-get update
candidate="$(apt-cache policy python3-pip | awk '/Candidate:/ { print $2; exit }')"
if [[ -z "$candidate" || "$candidate" == "(none)" ]]; then
    echo "python3-pip has no installable candidate after apt-get update" >&2
    exit 67
fi

{
    echo "python3-pip_candidate=${candidate}"
    echo "--- apt-cache policy python3-pip ---"
    apt-cache policy python3-pip
    echo "--- apt-cache depends python3-pip ---"
    apt-cache depends python3-pip
} | tee "$output_dir/apt-resolution.txt"

apt-get install --download-only --yes --no-install-recommends "python3-pip=${candidate}"
shopt -s nullglob
packages=(/var/cache/apt/archives/*.deb)
if (( ${#packages[@]} == 0 )); then
    echo "apt reported success but downloaded no .deb packages" >&2
    exit 68
fi
cp "${packages[@]}" "$output_dir/"

{
    echo -e "package\tversion\tarchitecture\tfilename"
    for package in "$output_dir"/*.deb; do
        printf '%s\t%s\t%s\t%s\n' \
            "$(dpkg-deb -f "$package" Package)" \
            "$(dpkg-deb -f "$package" Version)" \
            "$(dpkg-deb -f "$package" Architecture)" \
            "$(basename "$package")"
    done | LC_ALL=C sort
} > "$output_dir/apt-bundle-packages.tsv"

echo "downloaded ${#packages[@]} packages into $output_dir"
