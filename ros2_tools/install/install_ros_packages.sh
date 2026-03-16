#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

DIR="$(cd "$(dirname "$0")" && pwd)"

case "${1:-}" in
  install)
    DISTRO="${2:-}"
    if [[ -z "$DISTRO" ]]; then
      echo "Usage: $0 install <ros-distro>  e.g. ./install_ros_packages.sh install humble"
      exit 1
    fi

    # make apt non-interactive and quiet about config-file prompts
    export DEBIAN_FRONTEND=noninteractive
    export APT_LISTCHANGES_FRONTEND=none

    # ensure needrestart won't prompt (auto-accept restarts)
    # echo '$nrconf{restart} = "a";' | sudo tee /etc/needrestart/conf.d/99-auto.conf >/dev/null

    # update + upgrade non-interactively
    sudo apt-get update
    sudo DEBIAN_FRONTEND=noninteractive apt-get -y upgrade \
      -o Dpkg::Options::="--force-confdef" \
      -o Dpkg::Options::="--force-confold"

    # install packages in grouped calls (fewer apt invocations)
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
      -o Dpkg::Options::="--force-confdef" \
      -o Dpkg::Options::="--force-confold" \
      ros-"$DISTRO"-ros2-control \
      ros-"$DISTRO"-ros2-controllers \
      ros-"$DISTRO"-rplidar-ros \
      ros-"$DISTRO"-slam-toolbox \
      ros-"$DISTRO"-navigation2 \
      ros-"$DISTRO"-nav2-bringup \
      ros-"$DISTRO"-xacro \
      ros-"$DISTRO"-imu-tools \
      ros-"$DISTRO"-tf2-tools \
      ros-"$DISTRO"-tf-transformations \
      ros-"$DISTRO"-robot-localization \
      ros-"$DISTRO"-image-geometry \
      libserial-dev \
      software-properties-common \
      ros-"$DISTRO"-twist-mux \
      ros-"$DISTRO"-spatio-temporal-voxel-layer \
      ros-"$DISTRO"-gazebo-ros2-control \
      ros-"$DISTRO"-gazebo-ros-pkgs

    # packages with wildcard pattern (keep as separate install so shell expansion behavior is predictable)
    # Note: the shell will not expand these unless matching files exist in current dir.
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
      -o Dpkg::Options::="--force-confdef" \
      -o Dpkg::Options::="--force-confold" \
      "ros-${DISTRO}-librealsense*" "ros-${DISTRO}-realsense2-*" || true

    # copy udev rule if missing (fix path usage)
    if [[ ! -f "/etc/udev/rules.d/99-realsense-libusb.rules" ]]; then
      if [[ -f "$DIR/99-realsense-libusb.rules" ]]; then
        sudo cp "$DIR/99-realsense-libusb.rules" /etc/udev/rules.d/
        sudo udevadm control --reload-rules && sudo udevadm trigger
      else
        echo "Warning: udev rule file not found at $DIR/99-realsense-libusb.rules — skipping copy"
      fi
    fi

    echo "Package install finished."
    ;;

  *)
    printf "Type install then the ros distro to install the common ros packages for robot development\n\nexample:\n\t./install_ros_packages.sh install humble\n\n"
    ;;
esac