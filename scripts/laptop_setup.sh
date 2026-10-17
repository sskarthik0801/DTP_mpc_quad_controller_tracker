#!/usr/bin/env bash
#
# One-time setup for a LAPTOP that will visualize the Jetson's M1 stack in RViz.
# Installs ROS 2 Humble (desktop, incl. RViz) on Ubuntu 22.04. Needs sudo + internet.
#
# NOTE: there is nothing to colcon-build for viewing. The topics we display
# (/cloud_registered, /lidar_slam/odom, rog_map/*, /tf) are all standard ROS 2
# message types, so base ROS 2 + RViz is enough.
#
# Ubuntu 22.04 only. On Windows/macOS use WSL2 or a ros:humble Docker image instead.
#
set -euo pipefail

if [ -d /opt/ros/humble ]; then
  echo "ROS 2 Humble already present at /opt/ros/humble — nothing to do."
  echo "Next: bash scripts/laptop_viewer.sh   (follower view: bash scripts/laptop_dtp_viewer.sh)"
  exit 0
fi

. /etc/os-release
if [ "${UBUNTU_CODENAME:-}" != "jammy" ]; then
  echo "WARNING: this installer targets Ubuntu 22.04 (jammy); you have '${UBUNTU_CODENAME:-unknown}'." >&2
  echo "         For other systems use a ros:humble Docker image or WSL2." >&2
fi

echo "==> enabling universe + adding the ROS 2 apt repo"
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

echo "==> installing ros-humble-desktop (this is large)"
sudo apt update
sudo apt install -y ros-humble-desktop

echo
echo "Done. Next: bash scripts/laptop_viewer.sh   (follower view: bash scripts/laptop_dtp_viewer.sh)"
