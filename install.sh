#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# Pick and place 실습 (ROS 1 Noetic) — 사전 설치 스크립트
#
#   전제: Ubuntu 20.04 (데스크톱) · 인터넷 · sudo 비밀번호 — ROS 가 없으면 이 스크립트가 설치한다
#   사용:  cd ~/edu_pick_ros1 && bash install.sh
#   끝나면: bash check_setup.sh  (설치가 제대로 됐는지 자동 점검)
#   몇 번 다시 실행해도 된다 — 이미 된 단계는 건너뛴다.
#
#   하는 일
#     [0] ROS Noetic 설치 (없을 때만 · 공식 apt 저장소 · 약 1 GB 내려받음)
#     [1] apt 패키지 설치 (MoveIt · RealSense 드라이버 · OpenCV …) + RealSense USB 권한(udev) 규칙
#     [2] RB5 로봇 모델(레인보우 rbpodo_description) 받기 → ROS 1 패키지로 감싸기
#     [3] 워크스페이스 빌드 (catkin_make) · ~/.bashrc 에 환경 설정 줄 추가
#
#   옵션:  SKIP_APT=1  apt 단계를 건너뜀(이미 다 설치된 PC)
#          NO_BASHRC=1 ~/.bashrc 에 source 줄을 추가하지 않음
# ─────────────────────────────────────────────────────────────────────────────
set -euo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 버전 고정 — RB5 모델 파일을 가져오는 레인보우 저장소 커밋 (ROS 2 판과 같은 모델)
RBPODO_ROS2_COMMIT="5e8294a985e7ce5e20e70564c2681130afc5f502"

step() { echo; echo "━━━ $* ━━━"; }
die()  { echo "❌ $*" >&2; exit 1; }

[ "$(id -u)" -ne 0 ] || die "sudo 없이 일반 사용자로 실행하세요: bash install.sh"
. /etc/os-release
[ "${VERSION_CODENAME:-}" = "focal" ] || die "Ubuntu 20.04(focal) 가 아닙니다(지금: ${PRETTY_NAME:-알 수 없음}). ROS 1 Noetic 은 20.04 에서만 동작합니다."
case "$WS" in *" "*) die "설치 폴더 경로에 빈칸이 있습니다: $WS — 홈 폴더 바로 아래 edu_pick_ros1 로 옮기세요." ;; esac

# ── [0] ROS Noetic ───────────────────────────────────────────────────────────
# 공식 설치 절차(wiki.ros.org/noetic/Installation/Ubuntu)를 따른다. 저장소 키는 ros-apt-source 패키지로 받는다.
step "[0/3] ROS 1 Noetic"
if [ -f /opt/ros/noetic/setup.bash ]; then
  echo "  이미 설치됨 — 건너뜀"
elif [ "${SKIP_APT:-0}" = "1" ]; then
  die "ROS Noetic 이 없는데 SKIP_APT=1 입니다."
else
  echo "  ROS Noetic 을 설치합니다(약 1 GB · 인터넷 속도에 따라 10~30분)."
  sudo apt-get update
  sudo apt-get install -y curl lsb-release
  ROS_APT_SOURCE_VERSION=$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
                           | grep -F '"tag_name"' | awk -F'"' '{print $4}')
  [ -n "$ROS_APT_SOURCE_VERSION" ] || die "ROS 저장소 정보를 받지 못했습니다 — 인터넷 연결을 확인하세요."
  curl -fsSL -o /tmp/ros-apt-source.deb \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros-apt-source_${ROS_APT_SOURCE_VERSION}.focal_all.deb"
  sudo dpkg -i /tmp/ros-apt-source.deb
  sudo apt-get update
  sudo apt-get install -y ros-noetic-desktop
  [ -f /opt/ros/noetic/setup.bash ] || die "ROS Noetic 설치에 실패했습니다 — 위 메시지를 확인하세요."
fi

# ── [1] apt ──────────────────────────────────────────────────────────────────
APT_PKGS=(
  ros-noetic-moveit
  ros-noetic-xacro ros-noetic-joint-state-publisher ros-noetic-joint-state-publisher-gui
  ros-noetic-robot-state-publisher ros-noetic-cv-bridge ros-noetic-tf2-ros
  ros-noetic-realsense2-camera ros-noetic-rqt-image-view
  python3-opencv python3-numpy python3-yaml python3-catkin-pkg
  git curl build-essential
)
step "[1/3] apt 패키지 + RealSense USB 권한"
if [ "${SKIP_APT:-0}" = "1" ]; then
  echo "  SKIP_APT=1 — 건너뜀"
else
  MISSING=()
  for p in "${APT_PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || MISSING+=("$p"); done
  if [ ${#MISSING[@]} -eq 0 ]; then
    echo "  전부 설치돼 있음"
  else
    echo "  설치할 패키지: ${MISSING[*]}"
    sudo apt-get update
    sudo apt-get install -y "${MISSING[@]}"
  fi
  # RealSense 카메라를 sudo 없이 쓰기 위한 USB 권한 규칙 (없으면 실습실에서 카메라가 안 잡힌다)
  if [ ! -f /etc/udev/rules.d/99-realsense-libusb.rules ]; then
    curl -fsSL -o /tmp/99-realsense-libusb.rules \
      https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
    sudo cp /tmp/99-realsense-libusb.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules 2>/dev/null && sudo udevadm trigger 2>/dev/null || true
    echo "  RealSense USB 권한 규칙 설치"
  else
    echo "  RealSense USB 권한 규칙 이미 있음"
  fi
fi

# ── [2] RB5 로봇 모델 ────────────────────────────────────────────────────────
# 레인보우 공식 모델(Apache-2.0)은 ROS 2 저장소에만 있다. 모델 파일(xacro·메시)만 가져와
# ROS 1 패키지(rbpodo_description)로 감싼다 → URDF 의 $(find rbpodo_description) 가 그대로 동작.
step "[2/3] RB5 로봇 모델 (rbpodo_description)"
DESC="$WS/src/thirdparty/rbpodo_description"
if [ -f "$DESC/package.xml" ] && grep -q "<buildtool_depend>catkin" "$DESC/package.xml"; then
  echo "  이미 있음 — 건너뜀"
else
  TMP="$(mktemp -d)"
  git clone -q --filter=blob:none --no-checkout https://github.com/RainbowRobotics/rbpodo_ros2.git "$TMP/rbpodo_ros2"
  git -C "$TMP/rbpodo_ros2" sparse-checkout set rbpodo_description
  git -C "$TMP/rbpodo_ros2" checkout -q "$RBPODO_ROS2_COMMIT"
  mkdir -p "$WS/src/thirdparty"
  rm -rf "$DESC"
  mkdir -p "$DESC"
  cp -r "$TMP/rbpodo_ros2/rbpodo_description/robots" "$TMP/rbpodo_ros2/rbpodo_description/meshes" "$DESC/"
  cp "$TMP/rbpodo_ros2/rbpodo_description/LICENSE" "$DESC/" 2>/dev/null || true
  rm -rf "$TMP"
  cat > "$DESC/package.xml" <<'XML'
<?xml version="1.0"?>
<package format="2">
  <name>rbpodo_description</name>
  <version>0.0.0</version>
  <description>RainbowRobotics rbpodo_description (xacro·메시) — install.sh 가 ROS 1 패키지로 감쌈</description>
  <maintainer email="none@example.com">RainbowRobotics</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
XML
  cat > "$DESC/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.0.2)
project(rbpodo_description)
find_package(catkin REQUIRED)
catkin_package()
install(DIRECTORY robots meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
CMAKE
  echo "  받음: $DESC (커밋 ${RBPODO_ROS2_COMMIT:0:7})"
fi

# ── [3] 빌드 ─────────────────────────────────────────────────────────────────
step "[3/3] catkin_make"
cd "$WS"
set +u; source /opt/ros/noetic/setup.bash; set -u
catkin_make
set +u; source "$WS/devel/setup.bash"; set -u
for p in edu_pick_msgs edu_pick_description edu_pick_driver edu_pick_moveit edu_pick_perception edu_pick_task \
         edu_pick_bringup rbpodo_description; do
  rospack find "$p" >/dev/null 2>&1 || die "패키지 '$p' 를 찾을 수 없습니다 — 위 catkin_make 로그를 확인하세요."
done

# 새 터미널에서 자동으로 이 워크스페이스를 쓰게 한다 (NO_BASHRC=1 이면 건너뜀)
LINE="source $WS/devel/setup.bash"
if [ "${NO_BASHRC:-0}" != "1" ] && ! grep -qF "$LINE" ~/.bashrc; then
  echo "$LINE" >> ~/.bashrc
  echo "  ~/.bashrc 에 추가: $LINE"
fi

echo
echo "✅ 설치 완료. 새 터미널을 열거나 'source ~/.bashrc' 후 점검하세요:"
echo "     bash $WS/check_setup.sh"
