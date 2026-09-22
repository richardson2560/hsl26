#!/bin/bash

DIR_WITH_RULES="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "[INFO] Setting up udev rules..."

# Kobuki
sudo cp $DIR_WITH_RULES/30-kobuki.rules /etc/udev/rules.d

echo "[INFO] Restarting udev service..."
sudo service udev reload
sudo service udev restart

echo "Finished"
