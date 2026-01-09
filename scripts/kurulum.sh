#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ROS2 bulunamadi. Lutfen ROS2 Humble kurun."
  exit 1
fi

sudo apt-get update
sudo apt-get install -y python3-pip
pip3 install --user -r requirements.txt

echo "Kurulum tamam."
