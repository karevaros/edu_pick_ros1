#!/usr/bin/env python3
"""자세 이동·관절 조그 도구 — 실기 첫 구동 확인, 수업 전 자세 맞추기용.

    rosrun edu_pick_task go_pose.py home              # scene.yaml 의 이름 있는 자세로 (home / observe)
    rosrun edu_pick_task go_pose.py observe
    rosrun edu_pick_task go_pose.py --jog wrist3 5    # 관절 하나를 +5° (작게 움직여 보기)
    rosrun edu_pick_task go_pose.py --show            # 지금 관절값(도) 출력만
    rosrun edu_pick_task go_pose.py --grip 100        # 그리퍼 벌림 % (100 = 최대 10 cm · 0 = 닫힘)
    rosrun edu_pick_task go_pose.py home --yes        # 확인 없이 실행 (가상 모드 전용으로 쓸 것)

기본은 '계획 → RViz 에 경로 표시 → Enter → 실행' (pick_place --step 과 같은 방식).
"""
import argparse
import math
import sys
import time

import rospy

from edu_pick_task.pick_place import ARM_JOINTS, GRIPPER_OPEN, PickPlace, StepFailed

MAX_JOG_DEG = 30.0


def main():
    ap = argparse.ArgumentParser(description="자세 이동·관절 조그")
    ap.add_argument("pose", nargs="?", help="home / observe")
    ap.add_argument("--jog", nargs=2, metavar=("JOINT", "DEG"), help="관절 하나를 DEG 도 만큼")
    ap.add_argument("--show", action="store_true", help="지금 관절값만 출력")
    ap.add_argument("--grip", type=float, metavar="PCT", help="그리퍼 벌림 %% (0~100)")
    ap.add_argument("--yes", action="store_true", help="확인 없이 실행")
    args = ap.parse_args(rospy.myargv()[1:])
    if not (args.pose or args.jog or args.show or args.grip is not None):
        ap.error("자세 이름, --jog, --grip, --show 중 하나를 주세요")

    rospy.init_node("go_pose", disable_signals=True)
    node = PickPlace(step_mode=not args.yes)
    code = 0
    try:
        t0 = time.time()
        while not all(j in node.joints for j in ARM_JOINTS):
            if time.time() - t0 > 20:
                raise StepFailed("/joint_states 가 들어오지 않음 — bringup 이 떠 있나요?")
            time.sleep(0.1)
        cur = [node.joints[j] for j in ARM_JOINTS]
        print("지금 관절[deg]: " + "  ".join(f"{j} {math.degrees(v):7.2f}" for j, v in zip(ARM_JOINTS, cur)))
        if args.show:
            return
        if args.grip is not None:
            pct = min(100.0, max(0.0, args.grip))
            print(f"그리퍼 벌림 {pct:.0f} %")
            node.gripper(GRIPPER_OPEN * pct / 100.0, f"그리퍼 {pct:.0f}%")
            print("완료 — 명령 전송(RG2 는 도달 여부를 알려 주지 않으니 눈으로 확인)")
            return
        if args.jog:
            name, deg = args.jog[0], float(args.jog[1])
            if name not in ARM_JOINTS:
                raise StepFailed(f"관절 이름은 {ARM_JOINTS} 중 하나")
            if abs(deg) > MAX_JOG_DEG:
                raise StepFailed(f"조그는 ±{MAX_JOG_DEG:.0f}° 이내만 (요청 {deg}°)")
            target = list(cur)
            target[ARM_JOINTS.index(name)] += math.radians(deg)
            what = f"{name} {deg:+.1f}°"
        else:
            poses = node.scene["poses"]
            if args.pose not in poses:
                raise StepFailed(f"scene.yaml poses 에 '{args.pose}' 없음 (있는 것: {list(poses)})")
            target = [math.radians(v) for v in poses[args.pose]]
            what = f"자세 '{args.pose}'"
        diff = [math.degrees(b - a) for a, b in zip(cur, target)]
        print(f"목표 {what} — 관절 변화[deg]: " + "  ".join(f"{j} {d:+7.2f}" for j, d in zip(ARM_JOINTS, diff)))
        node.move_joints(target, what)
        after = [node.joints[j] for j in ARM_JOINTS]
        err = max(abs(math.degrees(a - b)) for a, b in zip(after, target))
        print(f"완료 — 목표와 최대 차이 {err:.2f}°")
    except StepFailed as e:
        print(f"❌ 중단: {e}", flush=True)
        if node.active_goal is not None:
            print("   " + node.cancel_motion(), flush=True)
        code = 1
    except KeyboardInterrupt:
        print("\n중단(Ctrl+C) — " + node.cancel_motion(), flush=True)
        code = 130
    finally:
        rospy.signal_shutdown("끝")
    sys.exit(code)


if __name__ == "__main__":
    main()
