#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

# Get the directory where THIS script lives (not where it's executed from)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# simple arg parsing
case "${1:-}" in
  --bare)
    ENV='bare'
    ;;
  --desktop)
    ENV='desktop'
    ;;
  *)
    echo "Usage: $0 --bare   # for bare-bones install"
    echo "       $0 --desktop # for desktop install"
    exit 1
    ;;
esac

# ensure UTF-8 locale
locale  # check current locale

sudo apt update
sudo apt install -y locales software-properties-common curl

# Generate and enable UTF-8
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# enable 'universe' repository
sudo add-apt-repository -y universe
sudo apt update

# get latest ros-apt-source release tag from GitHub
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F\" '{print $4}')

# determine ubuntu codename (focal, jammy, etc.)
UBU_CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-${VERSION_CODENAME}}")

# download the .deb for the current codename
TMP_DEB="/tmp/ros2-apt-source.deb"
curl -L -o "$TMP_DEB" \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBU_CODENAME}_all.deb"

# install repo package (fix deps if needed)
sudo dpkg -i "$TMP_DEB" || sudo apt -f install -y

sudo apt update
sudo apt upgrade -y

sudo apt install python3-pip -y

# install ROS packages depending on ENV
if [[ "${ENV}" == "bare" ]]; then
  sudo apt install -y ros-humble-ros-base
else
  sudo apt install -y ros-humble-desktop
fi

sudo apt install -y ros-dev-tools

# source ROS setup once (avoid duplicates in ~/.bashrc)
SETUP_LINE='source /opt/ros/humble/setup.bash'
if ! grep -Fxq "$SETUP_LINE" "$HOME/.bashrc"; then
  echo "$SETUP_LINE" >> "$HOME/.bashrc"
fi

# -------------------------
# Temporarily disable 'nounset' so ROS setup scripts can reference optional vars.
# This avoids "unbound variable" errors from upstream scripts while keeping
# 'set -u' active for the rest of this installer.
# -------------------------
set +u
# shellcheck disable=SC1090
source /opt/ros/humble/setup.bash
set -u

# add current user to dialout group
sudo usermod -aG dialout "$USER"

# Run additional ROS package installer relative to this script
if [[ -f "$SCRIPT_DIR/install/install_ros_packages.sh" ]]; then
  sudo bash "$SCRIPT_DIR/install/install_ros_packages.sh" install humble
else
  echo "Warning: install/install_ros_packages.sh not found in $SCRIPT_DIR"
fi

# source ~/.bashrc so current shell gets updates (optional)
# shellcheck disable=SC1090
source ~/.bashrc

echo "Done."