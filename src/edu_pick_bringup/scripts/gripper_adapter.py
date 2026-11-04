#!/usr/bin/env python3
"""RG2 그리퍼 어댑터 — 표준 ROS 액션을 레인보우 제어박스 명령으로 번역한다.

■ 왜 필요한가
  RG2 는 ros2_control 관절이 아니다. 레인보우 제어박스가 그리퍼를 물고 있고,
  제어박스 스크립트 명령 `gripper_macro …` 로만 움직인다.
  이 노드는 일반 팔 컨트롤러와 **같은 모양의 액션**(FollowJointTrajectory)을 열어 두고,
  목표 관절각을 받으면 제어박스 명령으로 바꿔 보낸다. 그래서 실습 노드는 그리퍼가
  무엇이든 똑같이 액션만 부르면 된다.

    /gripper_controller/follow_joint_trajectory  (액션 서버)  ← 실습 노드가 부른다
    /gripper_controller/joint_states             (토픽)       → 손가락 TF 표시용
    명령은 rb_driver 의 /rb_driver/script 서비스로 제어박스에 보낸다(명령 채널을 팔과 함께 쓴다).

■ 명령 형식 (실기 펜던트 스크립트에서 확인, 2026-10-29 · 2026-11-03 사진 재확인)
    gripper_macro 12,0,0,0,0,0,0,0,0,0           ← 기능 0 = 초기화 (첫 명령 전에 한 번)
    gripper_macro 12,0,1,0,<벌림%>,<힘%>,0,0,0,0 ← 기능 1 = 구동
    = 모델 12(OnRobot RG2) · 연결점 0(툴 플랜지) · 기능 · 0 · 벌림[%] · 힘[%] · 0×4
    펜던트 예: 12,0,1,0,48,54,… (벌림 48 % · 힘 54 %) / 12,0,1,0,100,100,… (최대 벌림)

■ 관절각 ↔ 벌림 %
    rg2_finger_joint1 = 0.0 rad → 0 %(닫힘),  0.6 rad → 100 %(열림)
    (모델 관절 상한은 1.18 이지만 실기 100 % 는 0.6 rad 에 해당 — 카메라 영상으로 확인)

■ ⚠ '잡혔는지'는 알 수 없다
  제어박스가 명령을 '접수'했다는 응답만 온다. 그래서 손가락 이동 거리에 비례해 기다린 뒤
  (settle_base + 이동 거리 / speed_mps) 성공으로 본다. 실제로 잡았는지는 실습 노드가 카메라로 확인한다.

■ 초기화
  RG2 는 첫 구동 전에 초기화 명령이 필요하고, 초기화가 끝나기 전에 온 명령은 무시된다.
  그래서 시작하자마자 초기화 → init_wait 기다림 → 100 % 열기를 해 둔다(init_on_start).

■ dry_run:=true (가짜 하드웨어 모드)
  명령을 보내지 않고 로그만 남긴다. 손가락 관절 상태는 그대로 바뀌므로 RViz 에서 움직임이 보인다.
"""
import os
import threading
import time

import actionlib
import rospkg
import rospy
import yaml
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState

from edu_pick_msgs.srv import RobotScript

JOINT, MIMIC = "rg2_finger_joint1", "rg2_finger_joint2"


class GripperAdapter:
    def __init__(self):
        # 기본값 = scene.yaml 의 gripper 절 (같은 이름의 ~파라미터가 있으면 그것이 우선)
        scene_file = rospy.get_param("~scene_file", os.path.join(
            rospkg.RosPack().get_path("edu_pick_description"), "config", "scene.yaml"))
        with open(scene_file) as f:
            sg = yaml.safe_load(f)["gripper"]
        sg["stroke_m"] = sg["max_open"]
        g = lambda n, d: rospy.get_param("~" + n, sg.get(n, d))
        self.dry = bool(g("dry_run", False))
        self.script_srv = g("script_service", "/rb_driver/script")
        self.j_closed, self.j_open = float(g("joint_closed", 0.0)), float(g("joint_open", 0.6))  # 0 % / 100 %
        self.force_pct = int(g("force_pct", 60))              # 힘 [%]
        self.settle_base = float(g("settle_base", 0.4))       # 명령 뒤 대기 = settle_base + 이동 거리 / speed_mps [s]
        self.speed_mps = float(g("speed_mps", 0.08))          # 손가락 속도 가정값 [m/s]
        self.settle_sec = float(g("settle_sec", 2.5))         # speed_mps 가 0 일 때의 고정 대기 [s]
        self.stroke_m = float(g("stroke_m", 0.10))            # 0 ~ 100 % 의 손가락 벌림 [m]
        self.cmd_move = g("cmd_move", "gripper_macro 12,0,1,0,{ratio_pct:.0f},{force_pct},0,0,0,0")
        self.cmd_init = g("cmd_init", "gripper_macro 12,0,0,0,0,0,0,0,0,0")   # 빈 값이면 보내지 않음
        self.init_wait = float(g("init_wait", 3.0))           # 초기화 뒤 기다리는 시간 [s]
        init_on_start = bool(g("init_on_start", True))        # false = 시작할 때 초기화·열기를 하지 않음

        self._pos = self.j_open
        self._initialized = False
        self._init_lock = threading.Lock()
        self._call = None
        if not self.dry:
            self._call = rospy.ServiceProxy(self.script_srv, RobotScript)

        self._js = rospy.Publisher("joint_states", JointState, queue_size=10)
        rospy.Timer(rospy.Duration(1 / 30), self._pub_js)
        self.server = actionlib.SimpleActionServer("follow_joint_trajectory", FollowJointTrajectoryAction,
                                                   self._on_goal, auto_start=False)
        self.server.start()
        if not self.dry and init_on_start:
            threading.Thread(target=self._startup_init, daemon=True).start()
        rospy.loginfo("그리퍼 어댑터 시작" + ("  [dry_run: 명령을 보내지 않음]" if self.dry else ""))

    def _ensure_init(self):
        """초기화를 한 번만 한다(시작 스레드와 첫 목표가 겹쳐도 한 번). 성공하면 True."""
        with self._init_lock:
            if self._initialized or not self.cmd_init:
                return True
            if not self._send(self.cmd_init):
                return False
            time.sleep(self.init_wait)
            self._initialized = True
            return True

    def _startup_init(self):
        try:
            rospy.wait_for_service(self.script_srv, timeout=60.0)
        except rospy.ROSException:
            rospy.logwarn("rb_driver 스크립트 서비스가 60초 안에 뜨지 않아 시작 초기화를 건너뜀 — 첫 명령 때 초기화")
            return
        time.sleep(2.0)                                   # 팔 드라이버가 자리 잡을 시간
        if self._ensure_init() and self._send(self._move_cmd(1.0)):
            self._pos = self.j_open
            rospy.loginfo("그리퍼 초기화 완료 → 100 % 열림")

    def _move_cmd(self, ratio):
        return self.cmd_move.format(ratio_pct=ratio * 100.0, force_pct=self.force_pct)

    def _ratio(self, pos):
        r = (pos - self.j_closed) / (self.j_open - self.j_closed)
        return min(1.0, max(0.0, r))

    def _send(self, script):
        if self.dry:
            rospy.loginfo(f"[dry_run] {script}")
            return True
        try:
            rospy.wait_for_service(self.script_srv, timeout=5.0)
            res = self._call(script=script)
        except (rospy.ROSException, rospy.ServiceException) as e:
            rospy.logerr(f"rb_driver 스크립트 서비스 호출 실패: {e}")
            return False
        if not res.success:
            rospy.logerr(f"제어박스가 명령을 받지 않음: {script} → {res.message}")
            return False
        rospy.loginfo(f"보냄: {script}")
        return True

    def _on_goal(self, goal):
        traj = goal.trajectory
        result = FollowJointTrajectoryResult()
        if JOINT not in traj.joint_names or not traj.points:
            result.error_code = result.INVALID_JOINTS
            self.server.set_aborted(result)
            return
        target = float(traj.points[-1].positions[traj.joint_names.index(JOINT)])
        if not (self._ensure_init() and self._send(self._move_cmd(self._ratio(target)))):
            result.error_code = result.INVALID_GOAL
            result.error_string = "제어박스가 그리퍼 명령을 받지 않음"
            self.server.set_aborted(result, result.error_string)
            return
        if self.speed_mps > 0:
            stroke = abs(self._ratio(target) - self._ratio(self._pos)) * self.stroke_m
            wait = self.settle_base + stroke / self.speed_mps
        else:
            wait = self.settle_sec
        time.sleep(wait)
        self._pos = target
        result.error_code = result.SUCCESSFUL
        self.server.set_succeeded(result)

    def _pub_js(self, _):
        m = JointState()
        m.header.stamp = rospy.Time.now()
        m.name = [JOINT, MIMIC]
        m.position = [self._pos, self._pos]
        self._js.publish(m)


def main():
    rospy.init_node("gripper_adapter")
    GripperAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
