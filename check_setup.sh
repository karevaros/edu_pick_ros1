#!/usr/bin/env bash
# ─────────────────────────────────────────────────────────────────────────────
# 설치 점검 (ROS 1 Noetic) — install.sh 다음에 실행. 마지막 줄이 "✅ 모두 통과" 면 실습 준비 끝.
#   bash ~/edu_pick_ros1/check_setup.sh
#   (로봇 없이 가상 모드로 시나리오 전체를 실제로 한 번 돌려 본다 — 약 1~2분)
# ─────────────────────────────────────────────────────────────────────────────
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PASS=0; FAIL=0; WARN=0
ok()   { echo "  ✅ $*"; PASS=$((PASS+1)); }
bad()  { echo "  ❌ $*"; FAIL=$((FAIL+1)); }
warn() { echo "  ⚠  $*"; WARN=$((WARN+1)); }

set +u
source /opt/ros/noetic/setup.bash 2>/dev/null || { echo "❌ ROS 1 Noetic 이 없습니다"; exit 1; }
source "$WS/devel/setup.bash" 2>/dev/null || { echo "❌ $WS/devel 이 없습니다 — install.sh 를 먼저 실행하세요"; exit 1; }

echo "[1] 환경"
grep -q "20.04" /etc/os-release && ok "Ubuntu 20.04" || warn "Ubuntu 20.04 가 아님 (미검증 환경)"
[ "$ROS_DISTRO" = "noetic" ] && ok "ROS_DISTRO=noetic" || bad "ROS_DISTRO=$ROS_DISTRO"

echo "[2] 패키지"
for p in rbpodo_description edu_pick_msgs edu_pick_description edu_pick_driver edu_pick_moveit \
         edu_pick_perception edu_pick_task edu_pick_bringup moveit_ros_move_group moveit_simple_controller_manager; do
  rospack find "$p" >/dev/null 2>&1 && ok "$p" || bad "$p 없음"
done
rospack find realsense2_camera >/dev/null 2>&1 && ok "realsense2_camera" \
  || warn "realsense2_camera 없음 — 실습실(실기)에서 필요: sudo apt install ros-noetic-realsense2-camera"

echo "[3] 모델"
xacro "$(rospack find edu_pick_description)/urdf/edu_pick.urdf.xacro" > /tmp/edu_pick_check.urdf 2>/dev/null \
  && ok "URDF 생성" || bad "URDF 생성 실패"
for f in tool_tip d405_link table rg2_hand link0 tcp; do
  grep -q "<link name=\"$f\"" /tmp/edu_pick_check.urdf && ok "링크 $f" || bad "링크 $f 없음"
done

echo "[4] 카메라 (실습실에서만 필요)"
if lsusb 2>/dev/null | grep -qi "8086:0b5b"; then ok "D405 USB 연결됨"; else warn "D405 연결 안 됨 (집에서는 정상)"; fi

echo "[5] 가상 모드로 시나리오 전체 실행 (1~2분)"
if [ "$FAIL" -gt 0 ]; then
  bad "앞 항목 실패로 건너뜀"
else
  LOG=/tmp/edu_pick_check_launch.log
  setsid roslaunch edu_pick_bringup pick_place.launch rviz:=false > "$LOG" 2>&1 &
  LPID=$!
  for i in $(seq 90); do grep -q "You can start planning" "$LOG" && break; sleep 1; done
  if ! grep -q "You can start planning" "$LOG"; then
    bad "move_group 이 뜨지 않음 — $LOG 확인"
  else
    ok "시스템 기동 (MoveIt·가상 하드웨어·가상 카메라)"
    OUT=$(timeout 240 rosrun edu_pick_task pick_place.py --count 1 2>&1)
    echo "$OUT" | grep -E "^\[|선택|✓|✕|완료|❌" | sed 's/^/      /'
    echo "$OUT" | grep -q "완료 — 1개 옮김" && ok "Pick and place 1회 성공" || bad "시나리오 실패 — 위 출력 확인"
  fi
  kill -INT -- -"$LPID" 2>/dev/null; sleep 5; kill -KILL -- -"$LPID" 2>/dev/null
fi

echo
echo "통과 $PASS · 경고 $WARN · 실패 $FAIL"
if [ "$FAIL" -eq 0 ]; then echo "✅ 모두 통과 — 실습 준비 완료"; else echo "❌ 실패 항목이 있습니다 — 위 메시지를 확인하세요"; exit 1; fi
