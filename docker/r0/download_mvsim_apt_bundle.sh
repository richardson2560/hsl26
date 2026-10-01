#!/usr/bin/env bash
set -euo pipefail

bundle_dir=${1:?usage: download_mvsim_apt_bundle.sh BUNDLE_DIR}
mkdir -p "$bundle_dir"
test -z "$(find "$bundle_dir" -mindepth 1 -maxdepth 1 -print -quit)"
apt-get update
apt-get install --download-only --yes --no-install-recommends ros-humble-mvsim
cp /var/cache/apt/archives/*.deb "$bundle_dir/"
dpkg-deb -f "$bundle_dir"/*.deb Package Version Architecture | paste - - - | sort > "$bundle_dir/packages.tsv"
sha256sum "$bundle_dir"/*.deb | sort > "$bundle_dir/sha256sums.txt"
