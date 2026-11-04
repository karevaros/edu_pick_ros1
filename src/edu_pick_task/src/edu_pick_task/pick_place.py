#!/usr/bin/env python3
"""Pick and place 시나리오 노드 — 실습의 주인공.

    rosrun edu_pick_task pick_place.py              # 연속 실행
    rosrun edu_pick_task pick_place.py --step       # 움직이기 직전마다 멈춤: 경로를 RViz 로 확인 → Enter (실기 권장)
    rosrun edu_pick_task pick_place.py --count 2    # 물체 2개를 차례로 옮김
    rosrun edu_pick_task pick_place.py --shape round  # 원형(캔·병 윗면·공·사과)만 집는다
    rosrun edu_pick_task pick_place.py --first-slot 1 # 하역 첫 칸이 이미 차 있을 때 둘째 칸부터

■ 시나리오 (12단계)
   1 초기위치      관절 목표  (MoveIt 계획)
   2 대기위치      관절 목표  — 카메라가 테이블을 내려다보는 자세
   3 물체인식      /edu_pick/objects 에서 집을 물체 고르기
   4 물체 위 이동  손끝 목표 자세 → 역기구학(IK) → 관절 목표   (그리퍼 미리 열기)
   5 하강          손끝 직선 이동 (Cartesian)
   6 잡기          그리퍼 닫기
   7 상승          손끝 직선 이동
   8 하역 위 이동  손끝 목표 자세 → IK → 관절 목표
   9 하강          손끝 직선 이동
  10 놓기          그리퍼 열기
  11 상승          손끝 직선 이동 (조금 들었을 때 손가락 사이가 비었는지 카메라로 확인)
  11+ 결과 확인    대기위치에서 다시 찍어, 집은 물체가 원래 자리에서 사라졌는지 본다
      → --count 로 여러 개를 옮기면 2 부터 반복
  12 초기위치      모두 끝난 뒤 한 번

■ 이 노드가 쓰는 ROS 인터페이스 (전부 MoveIt move_group 이 제공)
   서비스 /compute_ik  /compute_cartesian_path  /apply_planning_scene
   액션   /move_group  /execute_trajectory
   + /gripper_controller/follow_joint_trajectory (그리퍼 어댑터)

■ 손끝 프레임(tool_tip): z축 = 접근 방향, x축 = 손가락이 닫히는 방향.
   '위에서 아래로 집기' = tool_tip z축이 월드 -z 를 향하는 자세.
"""
import argparse
import contextlib
import csv
import datetime
import functools
import json
import math
import os
import sys
import time

import actionlib
import numpy as np
import rospkg
import rospy
import yaml
from tf2_ros import Buffer, TransformListener

from actionlib_msgs.msg import GoalID
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints, DisplayTrajectory,
                             ExecuteTrajectoryAction, ExecuteTrajectoryGoal, JointConstraint, MoveGroupAction,
                             MoveGroupGoal, MoveItErrorCodes, PlanningScene, RobotState, RobotTrajectory)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetCartesianPathRequest, GetPositionIK, \
    GetPositionIKRequest
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Int32, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from edu_pick_msgs.msg import DetectedObjectArray

ARM_JOINTS = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
GROUP, TIP, WORLD = "arm", "tool_tip", "world"
GRIPPER_OPEN = 0.6        # rg2_finger_joint1 [rad] = 100 % 벌림 (0 = 닫힘) — gripper_adapter 의 joint_open 과 같은 값
EARLIER_PLACED_SIZE = (0.07, 0.12)   # 앞 실행에서 놓은 물체(크기 모름) — 넉넉히 지름 7 cm · 높이 12 cm 로 본다


class StepFailed(Exception):
    pass


def timed(category, name=None):
    """메서드 실행 시간을 기록하는 장식자 — 분류(category)·이름은 timing.csv 로 간다."""
    def deco(fn):
        @functools.wraps(fn)
        def wrapper(self, *a, **kw):
            with self.timer(category, name or fn.__name__):
                return fn(self, *a, **kw)
        return wrapper
    return deco


def down_quat(yaw):
    """손끝이 아래(-z)를 보고, 닫히는 방향(x축)이 yaw 인 자세.  = Rz(yaw)·Rx(180°)"""
    return Quaternion(x=math.cos(yaw / 2), y=math.sin(yaw / 2), z=0.0, w=0.0)


class PickPlace:
    def __init__(self, step_mode=False, shape=None):
        scene_file = rospy.get_param("~scene_file", os.path.join(
            rospkg.RosPack().get_path("edu_pick_description"), "config", "scene.yaml"))
        with open(scene_file) as f:
            self.scene = yaml.safe_load(f)
        self.m = self.scene["motion"]          # 이동 설정
        self.g = self.scene["gripper"]         # 그리퍼 실측·설정
        self.rules = self.scene["grasp_rules"] # 물체별 잡는 법
        # '열기' 벌림. '잡기'는 물체마다 close_target() 으로 정한다
        self.grip_open = GRIPPER_OPEN * float(self.g["open_pct"]) / 100.0
        self.table_z = float(self.scene["table"]["top_z"])
        self.step_mode = step_mode
        self.shape_filter = shape        # None = 모양 무관 / "round" / "box"

        self.joints = {}
        self.active_goal = None
        # ── 시간 계측 ── 순서대로 일어난 일(timeline) + 인식 노드의 프레임별 처리 시간
        self.timeline = []            # dict(cycle, step, category, name, start, dur)
        self.det_frames = []          # 인식 노드가 보낸 프레임별 단계 시간(ms)
        self.cur_cycle, self.cur_step = 0, ""
        self.t_start = time.time()
        self.objects, self.objects_time = None, 0.0
        self.between, self.between_time = None, 0.0
        rospy.Subscriber("/edu_pick/detector_timing", String,
                         lambda m: self.det_frames.append(json.loads(m.data)), queue_size=50)
        rospy.Subscriber("/joint_states", JointState, self._on_js, queue_size=10)
        rospy.Subscriber("/edu_pick/objects", DetectedObjectArray, self._on_objects, queue_size=10)
        rospy.Subscriber("/edu_pick/between_fingers", Int32, self._on_between, queue_size=10)
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf)

        self.ik = rospy.ServiceProxy("/compute_ik", GetPositionIK)
        self.apply_scene = rospy.ServiceProxy("/apply_planning_scene", ApplyPlanningScene)
        self.obstacle_ids = set()     # MoveIt 장면에 넣어 둔 물체 이름
        self.placed = []              # 이미 옮겨 놓은 물체 (장애물로 계속 둔다)
        self.cart = rospy.ServiceProxy("/compute_cartesian_path", GetCartesianPath)
        self.move = actionlib.SimpleActionClient("/move_group", MoveGroupAction)
        self.exe = actionlib.SimpleActionClient("/execute_trajectory", ExecuteTrajectoryAction)
        self.grip = actionlib.SimpleActionClient("/gripper_controller/follow_joint_trajectory",
                                                 FollowJointTrajectoryAction)
        # 정지용 (cancel_motion)
        self.stop_pub = rospy.Publisher("/trajectory_execution_event", String, queue_size=10)
        self.display_pub = rospy.Publisher("/move_group/display_planned_path", DisplayTrajectory, queue_size=10)
        self.ctrl_cancel = rospy.Publisher("/arm_controller/follow_joint_trajectory/cancel", GoalID, queue_size=10)

    # ================================================================ 시간 계측
    @contextlib.contextmanager
    def timer(self, category, name):
        t0 = time.time()
        try:
            yield
        finally:
            self.timeline.append({"cycle": self.cur_cycle, "step": self.cur_step, "category": category,
                                  "name": name, "start": t0 - self.t_start, "dur": time.time() - t0})

    def save_timing(self, root):
        """timing.csv · detector_frames.csv · summary.txt 를 저장하고 요약을 출력한다."""
        d = os.path.join(os.path.expanduser(root), datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "timing.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["cycle", "step", "category", "name", "start", "dur"])
            w.writeheader()
            for r in self.timeline:
                w.writerow({**r, "start": f"{r['start']:.3f}", "dur": f"{r['dur']:.3f}"})
        stages = []
        for fr in self.det_frames:
            for k in fr["ms"]:
                if k not in stages:
                    stages.append(k)
        with open(os.path.join(d, "detector_frames.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["stamp", "objects"] + stages)
            for fr in self.det_frames:
                w.writerow([f"{fr['stamp']:.3f}", fr["objects"]] + [fr["ms"].get(k, "") for k in stages])
        lines = self.timing_summary(stages)
        with open(os.path.join(d, "summary.txt"), "w") as f:
            f.write("\n".join(lines) + "\n")
        print("\n".join(lines))
        print(f"\n시간 기록 저장: {d}")

    def timing_summary(self, stages):
        L = ["", "━━━ 시간 분석 ━━━"]
        seq = [r for r in self.timeline if r["category"] != "사람확인"]
        total = sum(r["dur"] for r in seq)
        wall = time.time() - self.t_start
        human = sum(r["dur"] for r in self.timeline if r["category"] == "사람확인")
        L.append(f"전체 경과 {wall:.1f} s · 사람 확인 대기 {human:.1f} s (분석에서 제외) · 기록된 작업 {total:.1f} s")
        cats = {}
        for r in seq:
            cats.setdefault(r["category"], [0.0, 0])
            cats[r["category"]][0] += r["dur"]
            cats[r["category"]][1] += 1
        L.append(f"{'분류':<10}{'합계[s]':>9}{'비율':>7}{'횟수':>6}{'평균[s]':>9}")
        for c, (t, n) in sorted(cats.items(), key=lambda kv: -kv[1][0]):
            L.append(f"{c:<10}{t:>9.2f}{100 * t / max(total, 1e-9):>6.0f}%{n:>6}{t / n:>9.2f}")
        cyc = {}
        for r in seq:
            if r["cycle"] > 0:
                cyc[r["cycle"]] = cyc.get(r["cycle"], 0.0) + r["dur"]
        if cyc:
            L.append("물체별(사이클) 작업 시간: " + " · ".join(f"{k}번 {v:.1f} s" for k, v in sorted(cyc.items())))
        if self.det_frames:
            L.append(f"인식 노드 프레임 {len(self.det_frames)}개 — 단계별 평균 [ms] (병렬로 계속 돈다):")
            for k in stages:
                vals = [fr["ms"][k] for fr in self.det_frames if k in fr["ms"]]
                if vals:
                    L.append(f"   {k:<12}{np.mean(vals):8.1f}  (중앙 {np.median(vals):.1f} · 최대 {np.max(vals):.1f})")
        return L

    # ================================================================ 기본 도구
    def _on_js(self, msg):
        for n, p in zip(msg.name, msg.position):
            self.joints[n] = p

    def _on_between(self, msg):
        self.between, self.between_time = msg.data, time.time()

    @timed("놓기확인")
    def held_points(self, timeout=3.0):
        """그리퍼를 연 뒤 손가락 사이에 보이는 점 개수(멈춘 뒤 들어온 값). 카메라가 없으면 None."""
        t_req = time.time() + 0.5
        while time.time() < t_req + timeout:
            if self.between is not None and self.between_time > t_req:
                return self.between
            time.sleep(0.05)
        return None

    def _on_objects(self, msg):
        self.objects, self.objects_time = msg, time.time()

    def call(self, srv, req, what, timeout=10.0):
        """서비스 호출 (서버가 없으면 기다리다 실패)."""
        try:
            srv.wait_for_service(timeout=timeout)
        except rospy.ROSException:
            raise StepFailed(f"{what}: {srv.resolved_name} 서비스 없음 — move_group 이 떠 있나요?")
        try:
            return srv(req)
        except rospy.ServiceException as e:
            raise StepFailed(f"{what}: 서비스 호출 실패 ({e})")

    def run_action(self, client, goal, timeout, what, category):
        """액션 목표를 보내고 결과를 기다린다. 걸린 시간은 category(경로계획·로봇동작·그리퍼)로 기록."""
        if not client.wait_for_server(rospy.Duration(5.0)):
            raise StepFailed(f"{what}: 액션 서버가 없습니다 — bringup 이 떠 있나요?")
        with self.timer(category, what):
            # ⚠ 정상으로 끝났을 때만 지운다. Ctrl+C·시간초과로 빠져나가면 남겨 둬야 cancel_motion 이
            #   이 목표를 취소할 수 있다.
            self.active_goal = client
            client.send_goal(goal)
            if not client.wait_for_result(rospy.Duration(timeout)):
                raise StepFailed(f"{what}: {timeout:.0f}초 안에 끝나지 않음")
            self.active_goal = None
            return client.get_result()

    def cancel_motion(self):
        """진행 중인 동작 정지.

        두 경로로 동시에 멈춘다 (ROS 2 판과 같은 방법):
            ① /trajectory_execution_event 에 "stop"  — MoveIt 이 실행을 중단 (MoveGroupInterface.stop() 과 같은 방법)
            ② arm_controller 액션의 목표 전부 취소   — 드라이버(rb_driver)가 제어박스에 task stop
          그리고 '응답'이 아니라 **관절값이 실제로 멈췄는지**로 확인한다."""
        self.stop_pub.publish(String(data="stop"))
        self.ctrl_cancel.publish(GoalID())             # 빈 GoalID = 모든 목표 취소
        if self.active_goal is not None:
            self.active_goal.cancel_all_goals()        # 진행 중인 액션(MoveIt·그리퍼)도 취소
        # 정지 확인: 0.5초 간격 두 번 관절값이 거의 같으면 멈춘 것
        time.sleep(0.7)
        a = np.array([self.joints.get(j, 0.0) for j in ARM_JOINTS])
        time.sleep(0.5)
        b = np.array([self.joints.get(j, 0.0) for j in ARM_JOINTS])
        moved = float(np.max(np.abs(b - a)))
        if moved < 0.002:
            return "동작을 정지했습니다(관절 정지 확인)."
        return f"⚠ 정지 확인 실패 — 0.5초 동안 관절이 {math.degrees(moved):.1f}° 움직였습니다. 비상정지 버튼!"

    def current_state(self):
        rs = RobotState()
        rs.joint_state.name = list(self.joints.keys())
        rs.joint_state.position = list(self.joints.values())
        return rs

    def tip_pose(self):
        t = self.tf.lookup_transform(WORLD, TIP, rospy.Time(0), rospy.Duration(1.0))
        p = Pose()
        p.position.x, p.position.y, p.position.z = (t.transform.translation.x,
                                                    t.transform.translation.y, t.transform.translation.z)
        p.orientation = t.transform.rotation
        return p

    # ================================================================ 동작 단위
    def move_joints(self, target, what):
        """관절 목표로 이동 — MoveIt 이 충돌 없는 경로를 계획하고 실행한다."""
        goal = MoveGroupGoal()
        r = goal.request
        r.group_name = GROUP
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        r.max_velocity_scaling_factor = float(self.m["velocity_scaling"])
        r.max_acceleration_scaling_factor = float(self.m["velocity_scaling"])   # 가속도도 같은 비율로
        r.start_state.is_diff = True
        c = Constraints()
        for n, v in zip(ARM_JOINTS, target):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=1e-4,
                                                       tolerance_below=1e-4, weight=1.0))
        r.goal_constraints = [c]
        goal.planning_options.plan_only = True          # ① 계획만 → ② 확인 → ③ 실행 (execute)
        # 계획은 무작위 탐색(RRTConnect)이라 가끔 실패하거나, 사후 검사에서 무효로 걸린다 → 최대 3번
        for attempt in range(1, 4):
            res = self.run_action(self.move, goal, 60.0, what, "경로계획")
            if res.error_code.val == MoveItErrorCodes.SUCCESS:
                break
            self.say(f"   ⚠ {what}: 경로 계획 실패 (오류 코드 {res.error_code.val}) — 다시 계획 {attempt}/3")
        else:
            raise StepFailed(f"{what}: 경로 계획 3번 실패")
        self.execute(res.planned_trajectory, what)

    def execute(self, traj, what):
        """계획된 궤적을 RViz 에 보여 주고(단계 모드면 확인을 받고) 실행한다."""
        jt = traj.joint_trajectory
        dur = jt.points[-1].time_from_start.to_sec() if jt.points else 0.0
        self.display_pub.publish(DisplayTrajectory(trajectory_start=self.current_state(), trajectory=[traj]))
        self.confirm(f"경로 계획 완료 ({len(jt.points)}점, {dur:.1f}초) — RViz '계획된 경로'를 확인하세요")
        goal = ExecuteTrajectoryGoal(trajectory=traj)
        res = self.run_action(self.exe, goal, dur * 2 + 30.0, what, "로봇동작")
        if res.error_code.val != MoveItErrorCodes.SUCCESS:
            raise StepFailed(f"{what}: 실행 실패 (오류 코드 {res.error_code.val})")

    @timed("경로계획", "IK")
    def solve_ik(self, pose):
        """손끝 목표 자세 → 관절값. 그리퍼는 180° 대칭이라 yaw 와 yaw+180° 둘 다 풀어 덜 도는 쪽을 쓴다."""
        best = None
        cur = np.array([self.joints[j] for j in ARM_JOINTS])
        x, y = pose.orientation.x, pose.orientation.y
        yaw = 2 * math.atan2(y, x)
        for cand_yaw in (yaw, yaw + math.pi):
            req = GetPositionIKRequest()
            req.ik_request.group_name = GROUP
            req.ik_request.ik_link_name = TIP
            req.ik_request.robot_state = self.current_state()      # 현재 관절값에서 출발(가까운 해)
            req.ik_request.avoid_collisions = True
            req.ik_request.pose_stamped.header.frame_id = WORLD
            req.ik_request.pose_stamped.pose.position = pose.position
            req.ik_request.pose_stamped.pose.orientation = down_quat(cand_yaw)
            req.ik_request.timeout = rospy.Duration(1.0)
            res = self.call(self.ik, req, "IK")
            if res.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            js = res.solution.joint_state
            q = np.array([js.position[js.name.index(j)] for j in ARM_JOINTS])
            cost = float(np.sum(np.abs(q - cur)))
            if best is None or cost < best[0]:
                best = (cost, q)
        if best is None:
            raise StepFailed(f"IK 해 없음 — 손끝 ({pose.position.x:.3f}, {pose.position.y:.3f}, "
                             f"{pose.position.z:.3f}) 에 닿을 수 없거나 충돌")
        return best[1]

    def move_above(self, x, y, z, yaw, what):
        """(x, y, z) 위치로 손끝을 내려보게 이동."""
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation = down_quat(yaw)
        self.move_joints(self.solve_ik(p), what)

    def move_linear(self, dz, what):
        """손끝을 현재 자세 그대로 수직으로 dz[m] 만큼 직선 이동 (+ 위 / - 아래)."""
        start = self.tip_pose()
        goal = Pose()
        goal.position.x, goal.position.y = start.position.x, start.position.y
        goal.position.z = start.position.z + dz
        goal.orientation = start.orientation

        req = GetCartesianPathRequest()
        req.header.frame_id = WORLD
        req.group_name, req.link_name = GROUP, TIP
        req.start_state = self.current_state()
        req.waypoints = [goal]
        req.max_step = 0.005            # 5 mm 마다 IK
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        with self.timer("경로계획", f"직선 경로({what})"):
            res = self.call(self.cart, req, what)
        if res.fraction < 0.99:
            raise StepFailed(f"{what}: 직선 경로를 {res.fraction * 100:.0f}% 밖에 못 만듦")
        traj = res.solution.joint_trajectory
        q = np.array([p.positions for p in traj.points])
        if len(q) > 1 and np.max(np.abs(np.diff(q, axis=0))) > 0.15:
            raise StepFailed(f"{what}: 직선 경로 중 관절이 갑자기 크게 돈다(손목 뒤집힘) — 실행 안 함")
        self._retime(traj, abs(dz))
        self.execute(RobotTrajectory(joint_trajectory=traj), what)

    def _retime(self, traj, dist):
        """직선 경로에 시간을 붙인다 — 부드럽게 출발·정지(코사인 속도).
        평균 속도 linear_speed, 단 어느 관절도 linear_joint_max 를 넘지 않게 전체 시간을 늘린다."""
        q = np.array([p.positions for p in traj.points])
        n = len(q)
        if n < 2:
            return
        T = max(dist / float(self.m["linear_speed"]), 1.0)
        vmax = float(self.m["linear_joint_max"])
        s = np.linspace(0.0, 1.0, n)
        for _ in range(20):
            t = T / math.pi * np.arccos(1 - 2 * s)          # s(t) = (1 - cos(πt/T)) / 2 의 역함수
            dt = np.diff(t)
            v = np.max(np.abs(np.diff(q, axis=0)) / np.maximum(dt[:, None], 1e-6))
            if v <= vmax:
                break
            T *= v / vmax * 1.05
        for p, ti in zip(traj.points, t):
            p.time_from_start = rospy.Duration.from_sec(float(ti))
            p.velocities, p.accelerations = [], []

    def gripper(self, pos, what):
        self.confirm(f"그리퍼 {'열기' if pos > 0.3 else '닫기'}")
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory(joint_names=["rg2_finger_joint1"],
                                          points=[JointTrajectoryPoint(positions=[pos],
                                                                       time_from_start=rospy.Duration(1.0))])
        res = self.run_action(self.grip, goal, 15.0, what, "그리퍼")
        if res is None or res.error_code != FollowJointTrajectoryResult.SUCCESSFUL:
            raise StepFailed(f"{what}: 그리퍼 실패 (오류 코드 {res.error_code})")

    # ================================================================ MoveIt 장면 (장애물)
    # MoveIt 은 URDF 에 있는 것(테이블·받침대·로봇 자신)만 안다. 카메라로 본 물체·이미 놓은 물체는
    # 직접 '장면(planning scene)'에 넣어 줘야 피해 간다. 쥔 물체는 그리퍼에 '붙여서' 함께 움직인다.
    PAD = 0.01   # 장애물 여유 [m]

    @staticmethod
    def _primitive(shape, width, length, height, pad):
        sp = SolidPrimitive()
        if shape == "round":
            sp.type, sp.dimensions = SolidPrimitive.CYLINDER, [height + pad, width / 2 + pad]
        else:
            sp.type, sp.dimensions = SolidPrimitive.BOX, [width + 2 * pad, length + 2 * pad, height + pad]
        return sp

    @timed("장면갱신")
    def _apply(self, ps, what):
        res = self.call(self.apply_scene, ps, what)
        if not res.success:
            raise StepFailed(f"{what}: MoveIt 장면 반영 실패")

    def set_obstacles(self, items):
        """장애물 목록으로 장면을 바꾼다. items: dict(id, shape, x, y, bottom, width, length, height, yaw)"""
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        for oid in self.obstacle_ids:
            ps.world.collision_objects.append(CollisionObject(id=oid, operation=CollisionObject.REMOVE))
        for it in items:
            co = CollisionObject(id=it["id"], operation=CollisionObject.ADD)
            co.header.frame_id = WORLD
            co.primitives = [self._primitive(it["shape"], it["width"], it["length"], it["height"], self.PAD)]
            p = Pose()
            p.position.x, p.position.y = float(it["x"]), float(it["y"])
            p.position.z = float(it["bottom"]) + (it["height"] + self.PAD) / 2
            yaw = 0.0 if it["shape"] == "round" or math.isnan(it["yaw"]) else it["yaw"]
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            co.primitive_poses = [p]
            ps.world.collision_objects.append(co)
        self._apply(ps, "장애물 갱신")
        self.obstacle_ids = {it["id"] for it in items}

    @staticmethod
    def as_item(o, oid=None):
        return {"id": oid or f"obstacle_{o.id}", "shape": o.shape, "x": o.top_center.x, "y": o.top_center.y,
                "bottom": o.top_center.z - o.height, "width": o.width, "length": o.length,
                "height": o.height, "yaw": o.yaw}

    def attach_held(self, obj, depth):
        """쥔 물체를 손끝(tool_tip)에 붙인다. depth = 손끝이 물체 윗면보다 내려간 깊이.
        tool_tip z축 = 접근(아래) 방향 → 물체 윗면은 z = -depth, 중심은 z = -depth + 높이/2"""
        a = AttachedCollisionObject(link_name=TIP,
                                    touch_links=["rg2_leftfinger", "rg2_rightfinger", "rg2_hand", TIP])
        a.object.id, a.object.operation = "held_object", CollisionObject.ADD
        a.object.header.frame_id = TIP
        # 여유 1.2 cm: 인식 길이가 실제보다 짧게 나올 수 있다(누운 병 인식 17 cm ↔ 실제 약 20 cm)
        a.object.primitives = [self._primitive(obj.shape, obj.width, obj.length, obj.height, 0.012)]
        p = Pose()
        p.position.z = -depth + obj.height / 2
        p.orientation.w = 1.0
        a.object.primitive_poses = [p]
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [a]
        self._apply(ps, "쥔 물체 붙이기")

    def detach_held(self, strict=True):
        """쥔 물체를 뗀다. strict=False: 붙은 게 없어도 괜찮다(시작할 때 정리용 —
        없는 물체를 지우라고 하면 MoveIt 은 '실패'를 돌려준다)."""
        a = AttachedCollisionObject(link_name=TIP)
        a.object.id, a.object.operation = "held_object", CollisionObject.REMOVE
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = [a]
        try:
            self._apply(ps, "쥔 물체 떼기")
        except StepFailed:
            if strict:
                raise
            return
        # MoveIt 은 떼어 낸 물체를 그 자리에 '세상 물체'로 남긴다 → 지운다(놓은 물체는 placed 로 따로 관리)
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.world.collision_objects = [CollisionObject(id="held_object", operation=CollisionObject.REMOVE)]
        try:
            self._apply(ps, "떼어 낸 물체 정리")
        except StepFailed:
            pass

    def close_target(self, width, squeeze):
        """잡기 목표 관절값: 물체 폭보다 squeeze 만큼 좁게 → 닿는 순간부터 설정한 힘으로 쥔다."""
        pct = min(100.0, 100.0 * max(0.0, width - squeeze) / float(self.g["max_open"]))
        return GRIPPER_OPEN * pct / 100.0, pct

    # ================================================================ 잡는 법 (물체 치수 → 깊이·조임)
    def classify(self, obj):
        """물체 종류를 '치수'로 가른다 (이름은 모른다). → "tall" / "lying" / "low" / "normal" """
        r, w, h = self.rules, obj.width, obj.height
        if h >= r["tall_ratio"] * w:
            return "tall"                                    # 세운 병
        if obj.shape == "box" and obj.length >= r["lying_length_ratio"] * w and h <= r["lying_height_ratio"] * w:
            return "lying"                                   # 누운 병 (위에서 보면 길쭉한 박스)
        if h < r["low_ratio"] * w:
            return "low"                                     # 사과·공·납작한 박스
        return "normal"                                      # 박스·캔

    def plan_grasp(self, obj):
        """잡는 손끝 높이·방향·조임 정하기.  반환 dict(kind, yaw, grip_z, advance, squeeze)

        깊이 = 손끝이 물체 윗면보다 내려가는 거리
          낮은 물체 = 가운데 높이(누운 원통을 축보다 위에서 잡으면 들 때 미끄러진다) · 그 밖 = grip_depth
        🔴 RG2 안쪽 스위치: 물체 윗면이 palm_depth 보다 깊이 들어가면 스위치가 눌려 그리퍼가 멈춘다
           → 깊이 상한 = palm_depth − palm_margin (키 큰 물체·누운 병은 더 얕게)"""
        kind = self.classify(obj)
        if math.isnan(obj.yaw):                     # 원형: 지금 손목 방향 그대로
            q = self.tip_pose().orientation
            yaw = 2 * math.atan2(q.y, q.x)
        else:                                       # 박스: 짧은 변 방향으로 닫는다
            yaw = obj.yaw
        low = obj.height < self.rules["low_ratio"] * obj.width      # 낮으면(누운 병 포함) 가운데 높이
        depth = obj.height * 0.5 if low else float(self.m["grip_depth"])
        max_depth = float(self.g["palm_depth"]) - float(self.g["palm_margin"])
        squeeze = float(self.g["close_squeeze"])
        if kind == "tall":
            max_depth = min(max_depth, float(self.rules["tall_max_insert"]))
            self.say("   → 키 큰 물체 — " + (f"손 안으로 {max_depth * 100:.1f} cm 까지만" if max_depth >= 0 else
                     f"윗면 {-max_depth * 100:.1f} cm 위에서 닫음(닫히며 손끝이 내려와 윗부분을 잡는다)"))
        elif kind == "lying":
            max_depth = min(max_depth, float(self.rules["lying_max_insert"]))
            squeeze = float(self.rules["lying_squeeze"])
            self.say(f"   → 누운 병으로 판단(길이 {obj.length * 100:.0f} cm · 폭 {obj.width * 100:.1f} cm) — "
                     f"얕게 {max_depth * 100:.1f} cm · 덜 조임 {squeeze * 100:.0f} cm")
        if depth > max_depth + 1e-4:
            self.say(f"   → 잡는 깊이 {depth * 100:.1f} cm → {max_depth * 100:.1f} cm 로 제한 (그리퍼 안쪽 스위치)")
            depth = max_depth
        # RG2 는 닫을수록 손끝이 앞(아래)으로 나온다 — 실측 2점(10 cm 벌림 20.5 cm · 0 cm 24 cm) 사이 직선 근사.
        # 물체 폭까지 닫혔을 때의 손끝도 테이블 위 min_tip_clearance 이상이어야 한다.
        advance = (self.g["tip_closed"] - self.g["tip_open"]) * (1.0 - min(obj.width / self.g["max_open"], 1.0))
        grip_z = max(obj.top_center.z - depth, self.table_z + float(self.m["min_tip_clearance"]) + advance)
        return {"kind": kind, "yaw": yaw, "grip_z": grip_z, "advance": advance, "squeeze": squeeze}

    # ================================================================ 인식
    @timed("인식대기")
    def detect(self, timeout=5.0):
        """대기위치에 멈춘 뒤 들어온 인식 결과만 쓴다(이동 중에 찍힌 결과는 버림)."""
        t_req = time.time() + 1.0                   # 멈춘 뒤 1초 이후 결과
        while time.time() < t_req + timeout:
            if self.objects is not None and self.objects_time > t_req:
                return list(self.objects.objects)
            time.sleep(0.05)
        raise StepFailed("인식 결과가 들어오지 않음 — /edu_pick/objects 를 확인하세요")

    def place_slots(self, n):
        """k 번째 물체를 놓을 xy — 하역 위치에서 step_xy 씩 옆으로."""
        pl = self.scene["place"]
        return [(pl["xy"][0] + k * pl["step_xy"][0], pl["xy"][1] + k * pl["step_xy"][1]) for k in range(n)]

    def choose(self, objects, slots):
        """집을 물체 고르기: 하역 구역 밖 · 집을 수 있는 것 중 테이블 중앙(roi 중심)에 가장 가까운 것."""
        roi = self.scene["detection"]
        cx, cy = sum(roi["roi_x"]) / 2, sum(roi["roi_y"]) / 2
        keep_out = float(self.scene["place"]["keep_out"])
        ok = []
        for o in objects:
            in_place = any(math.hypot(o.top_center.x - sx, o.top_center.y - sy) < keep_out for sx, sy in slots)
            why = "하역 구역(이미 옮긴 물체)" if in_place else o.note
            self.say(f"   · {o.id}: {o.shape:5s} 폭 {o.width * 100:4.1f} cm  높이 {o.height * 100:4.1f} cm  "
                     f"위치 ({o.top_center.x:.3f}, {o.top_center.y:.3f})" + (f"  ✕ {why}" if why else ""))
            if o.graspable and not in_place and (self.shape_filter is None or o.shape == self.shape_filter):
                ok.append(o)
        if not ok:
            return None
        return min(ok, key=lambda o: math.hypot(o.top_center.x - cx, o.top_center.y - cy))

    # ================================================================ 시나리오
    def say(self, text):
        print(text, flush=True)

    def step(self, no, name):
        self.cur_step = str(no)
        self.say(f"\n[{no:>3}] {name}")

    @timed("사람확인")
    def confirm(self, text):
        """단계 모드: 실제로 움직이기 직전에 멈추고 Enter 를 기다린다."""
        if self.step_mode:
            try:
                input(f"     ↳ {text}\n       Enter = 실행 / Ctrl+C = 중단 ")
            except EOFError:
                raise StepFailed("입력이 닫혀 실행하지 않고 중단합니다")

    def run(self, count=1, first_slot=0):
        poses = {k: [math.radians(v) for v in vals] for k, vals in self.scene["poses"].items()}
        all_slots = self.place_slots(first_slot + count)
        slots = all_slots[first_slot:]
        # 앞 칸에는 이전 실행에서 놓은 물체가 있다고 보고 장애물로 둔다 (카메라 인식 영역 밖이라 안 보인다)
        for i, (sx, sy) in enumerate(all_slots[:first_slot]):
            d, h = EARLIER_PLACED_SIZE
            self.placed.append({"id": f"earlier_{i}", "shape": "round", "x": sx, "y": sy, "bottom": self.table_z,
                                "width": d, "length": d, "height": h, "yaw": float("nan")})
        m = self.m

        self.say("MoveIt·컨트롤러·관절 상태 기다리는 중 …")
        t0 = time.time()
        while not all(j in self.joints for j in ARM_JOINTS):
            if time.time() - t0 > 20:
                raise StepFailed("/joint_states 가 들어오지 않음 — bringup 이 떠 있나요?")
            time.sleep(0.1)
        for c, n in ((self.ik, "/compute_ik"), (self.cart, "/compute_cartesian_path")):
            try:
                c.wait_for_service(timeout=20.0)
            except rospy.ROSException:
                raise StepFailed(f"{n} 서비스 없음 — move_group 이 떠 있나요?")

        self.detach_held(strict=False)     # 앞 실행이 남긴 장면 정리
        self.set_obstacles(list(self.placed))
        self.step(1, "초기위치로 이동")
        self.move_joints(poses["home"], "초기위치")

        done = 0
        for k in range(count):
            self.cur_cycle = k + 1
            self.step(2, "대기위치로 이동")
            self.move_joints(poses["observe"], "대기위치")

            self.step(3, "물체인식")
            seen = self.detect()
            with self.timer("위치계산", "물체 선택"):
                obj = self.choose(seen, all_slots)
            # 고른 물체 말고는 전부 장애물로: 인식한 다른 물체 + 이미 옮겨 놓은 물체
            others = [self.as_item(o) for o in seen if obj is None or o.id != obj.id]
            self.set_obstacles(others + self.placed)
            self.say(f"   → MoveIt 장면 장애물 {len(others) + len(self.placed)}개 (인식 {len(others)} · 옮겨 놓은 것 {len(self.placed)})")
            if obj is None:
                self.say("   → 집을 수 있는 물체가 없습니다. 종료합니다.")
                break
            with self.timer("위치계산", "잡는 높이·방향"):
                gp = self.plan_grasp(obj)
            yaw, grip_z, advance = gp["yaw"], gp["grip_z"], gp["advance"]
            above_z = obj.top_center.z + float(m["approach_height"])
            self.say(f"   → 선택: {obj.id} ({obj.shape}) · 집기 방향 {math.degrees(yaw):.0f}° · "
                     f"손끝 높이 {grip_z:.3f} m (닫으면 {advance * 100:.1f} cm 더 내려감 → 테이블 위 "
                     f"{(grip_z - advance - self.table_z) * 100:.1f} cm)")

            self.step(4, "물체 위로 이동 (그리퍼 열기)")
            self.gripper(self.grip_open, "그리퍼 열기")
            self.move_above(obj.top_center.x, obj.top_center.y, above_z, yaw, "물체 위 이동")

            self.step(5, f"하강 {(above_z - grip_z) * 100:.1f} cm")
            self.move_linear(grip_z - above_z, "하강")

            close_pos, close_pct = self.close_target(obj.width, gp["squeeze"])
            self.step(6, f"잡기 (물체 폭 {obj.width * 100:.1f} cm → 그리퍼 목표 {close_pct:.0f} %)")
            self.gripper(close_pos, "잡기")
            self.attach_held(obj, obj.top_center.z - grip_z)

            self.step(7, "상승")
            self.move_linear(above_z - grip_z, "상승")

            # 놓을 때 손끝 높이 = 잡을 때 손끝이 물체 바닥에서 떨어진 높이 + 여유
            tall = gp["kind"] == "tall"
            gap = float(m["release_gap_tall"] if tall else m["release_gap"])
            release_z = self.table_z + (grip_z - (obj.top_center.z - obj.height)) + gap
            if tall:
                self.say(f"   → 키 큰 물체(높이 {obj.height * 100:.0f} cm · 폭 {obj.width * 100:.1f} cm) — 테이블 위 {gap * 100:.0f} cm 에서 놓음")
            place_above = release_z + float(m["approach_height"])
            self.step(8, "하역 위치 위로 이동")
            place_xy = slots[k]
            self.say(f"   → 하역 위치 ({place_xy[0]:.2f}, {place_xy[1]:.2f})")
            # 박스형(긴 물체)은 정해진 방향으로 놓는다 — 원형은 방향이 상관없어 손목을 돌리지 않는다
            place_yaw = yaw if obj.shape == "round" else math.radians(float(self.scene["place"].get("yaw_deg", 0.0)))
            self.move_above(place_xy[0], place_xy[1], place_above, place_yaw, "하역 위치 이동")

            self.step(9, f"하강 {(place_above - release_z) * 100:.1f} cm")
            self.move_linear(release_z - place_above, "하강")

            # 놓을 때는 물체 폭 + release_margin 만큼만 연다 (옆 칸 물체를 손가락으로 치지 않게)
            rel_pct = min(float(self.g["open_pct"]),
                          100.0 * (obj.width + float(self.g["release_margin"])) / float(self.g["max_open"]))
            self.step(10, f"놓기 (그리퍼 {rel_pct:.0f} % 까지만 열기)")
            self.gripper(GRIPPER_OPEN * rel_pct / 100.0, "놓기")
            self.detach_held()

            # 놓기 확인: 조금 들고 손가락 사이를 본다. RG2 는 열렸는지 알려 주지 않는다
            #   (열기 명령이 접수됐는데 안 열려 물체를 쥔 채 돌아올 수 있다 → 다시 연다)
            lift = float(m["check_lift"])
            self.step(11, f"상승 ({lift * 100:.0f} cm 들고 놓였는지 확인)")
            self.move_linear(lift, "상승")
            thr = int(self.g["held_points"])
            for retry in range(3):
                n = self.held_points()
                if n is None or n < thr:
                    self.say(f"   ✓ 손가락 사이 비어 있음(점 {n}개)" if n is not None else "   (카메라 값 없음 — 확인 생략)")
                    break
                if retry == 2:
                    raise StepFailed(f"놓기 실패 — 그리퍼를 다시 열어도 물체가 손에 남아 있습니다(점 {n}개). 사람이 확인하세요")
                self.say(f"   ✕ 물체가 아직 손에 있음(점 {n}개) — 열기 다시 {retry + 1}/2")
                self.gripper(self.grip_open, "다시 열기")
            self.move_linear(place_above - release_z - lift, "상승")
            # 놓은 물체는 손이 빠져나온 '뒤에' 장애물로 등록한다(손가락이 둘러싼 채로 넣으면 시작부터 충돌)
            it = self.as_item(obj, oid=f"placed_{first_slot + k}")
            it.update(x=place_xy[0], y=place_xy[1], bottom=self.table_z)
            self.placed.append(it)
            self.set_obstacles(list(self.placed))

            self.step("11+", "결과 확인 (대기위치에서 다시 인식)")
            self.move_joints(poses["observe"], "대기위치")
            left = [o for o in self.detect()
                    if math.hypot(o.top_center.x - obj.top_center.x, o.top_center.y - obj.top_center.y) < 0.03]
            if left:
                self.say("   ✕ 물체가 원래 자리에 그대로 있습니다 — 잡기 실패")
            else:
                self.say("   ✓ 원래 자리에서 사라졌습니다 — 옮기기 성공")
                done += 1

        self.step(12, "초기위치로 이동")
        self.move_joints(poses["home"], "초기위치")
        self.set_obstacles([])
        self.say(f"\n완료 — {done}개 옮김")


def main():
    ap = argparse.ArgumentParser(description="Pick and place 시나리오")
    ap.add_argument("--step", action="store_true", help="단계마다 Enter 로 진행")
    ap.add_argument("--count", type=int, default=1, help="옮길 물체 개수")
    ap.add_argument("--shape", choices=["round", "box"], help="이 모양만 집는다")
    ap.add_argument("--log-dir", default="~/edu_pick_runs", help="시간 기록을 저장할 폴더")
    ap.add_argument("--first-slot", type=int, default=0,
                    help="하역 칸 시작 번호 (앞 칸에 이미 물체가 있을 때. 예: 1 → 둘째 칸부터)")
    args = ap.parse_args(rospy.myargv()[1:])

    # Ctrl+C 를 rospy 가 가로채면 통신이 먼저 끊겨 '정지 명령'을 못 보낸다 → 직접 처리(disable_signals)
    rospy.init_node("pick_place", disable_signals=True)
    node = PickPlace(step_mode=args.step, shape=args.shape)
    code = 0
    try:
        node.run(args.count, args.first_slot)
    except StepFailed as e:
        print(f"\n❌ 중단: {e}", flush=True)
        if node.active_goal is not None:          # 시간초과 등으로 빠져나왔으면 남은 동작도 멈춘다
            print("   " + node.cancel_motion(), flush=True)
        code = 1
    except KeyboardInterrupt:
        print("\n중단(Ctrl+C) — " + node.cancel_motion(), flush=True)
        code = 130
    finally:
        try:
            node.save_timing(args.log_dir)
        except Exception as e:
            print(f"(시간 기록 저장 실패: {e})")
        rospy.signal_shutdown("끝")
    sys.exit(code)


if __name__ == "__main__":
    main()
