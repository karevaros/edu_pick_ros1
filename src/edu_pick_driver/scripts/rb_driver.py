#!/usr/bin/env python3
"""레인보우 협동로봇(RB 시리즈) ROS 1 드라이버 — 제어박스와 TCP 로 직접 통신한다.

바탕: RainbowRobotics RB_ROS 의 rb_connector (BSD, github.com/makeitjune/RB_ROS) 의 통신 방식.
      C++/Qt4 코드를 파이썬으로 다시 쓰고, 실행할 궤적을 '액션 목표'에서 받도록 고쳤다
      (원본은 RViz 표시용 /move_group/display_planned_path 를 실행해, 직선 경로처럼 표시 없이
       보내는 궤적에서 엉뚱한 경로가 실행될 수 있었다).

■ 통신 (제어박스 IP 의 두 포트)
  명령 채널 5000   스크립트 한 줄을 보내면 "The command was executed" 로 접수를 알려 준다
  데이터 채널 5001  "reqdata" 를 보내면 '$' 로 시작하는 상태 묶음(관절값·로봇 상태·안전 신호)이 온다

■ 궤적 실행 = 궤적 전체를 제어박스에 넘긴다 (ROS 는 실행 중에 명령을 보내지 않는다)
    move_ros_j_clear()
    move_ros_j_add(시각[s], jnt[위치°×6], jnt[속도°/s×6], jnt[가속도°/s²×6])   ← 점마다
    move_ros_j_run(0, 점 개수)
  끝 판정: 로봇 상태가 '움직임(3)' 이 됐다가 '정지(1)' 로 돌아오고, 관절값이 마지막 점에 도착

■ 이 노드가 제공하는 것
  arm/joint_states                          (sensor_msgs/JointState)   팔 관절 6개
  arm_controller/follow_joint_trajectory    (액션)                      MoveIt 이 궤적을 보낸다
  rb_driver/script                          (edu_pick_msgs/RobotScript) 그리퍼 등 스크립트 한 줄
  rb_driver/status                          (edu_pick_msgs/RobotStatus) 제어박스 상태

■ 파라미터
  ~fake               true = 제어박스 없이 궤적을 시간에 맞춰 흉내낸다(가상 모드)
  ~robot_ip           제어박스 IP
  ~cb_simulation      true = 접속하면서 제어박스를 Simulation 모드로(팔 안 움직임) / false = Real
  ~speed_bar          접속하면서 거는 제어박스 속도바 (0~1)
  ~max_joint_velocity 이보다 빠른 관절이 있는 궤적은 실행하지 않는다 [rad/s]
"""
import math
import socket
import struct
import threading
import time

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState

from edu_pick_msgs.msg import RobotStatus
from edu_pick_msgs.srv import RobotScript, RobotScriptResponse

JOINTS = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]
CMD_PORT, DATA_PORT = 5000, 5001
ACK = "The command was executed"

# 상태 묶음에서 쓰는 값의 위치 [바이트] — 레인보우 SDK(rbpodo) SystemState 와 rb_connector 의 구조체가 같다
OFF_JNT_REF = 8            # float[6] 관절 기준값 [deg]
OFF_TASK_STATE = 332       # int
OFF_SPEED_BAR = 336        # float
OFF_ROBOT_STATE = 340      # int  1 = 정지 · 2 = 일시정지/사고 · 3 = 움직임
OFF_SIM_MODE = 404         # int  0 = Real · 1 = Simulation
OFF_OP_STAT = 444          # int[5] 충돌 · SOS · 자기충돌 · 소프트 비상정지 · 비상정지
OFF_INBOX_TRAP = 472       # int[2] 인박스 위반
STATE_MIN_BYTES = OFF_INBOX_TRAP + 8
IDLE, PAUSED, MOVING = 1, 2, 3


class ControlBox:
    """제어박스 두 채널. 명령은 한 번에 하나씩(잠금), 상태는 별도 스레드가 계속 읽는다."""

    def __init__(self, ip, rate_hz=50.0):
        self.ip = ip
        self.rate_hz = rate_hz
        self.cmd = self._connect(CMD_PORT)
        self.data = self._connect(DATA_PORT)
        self.cmd_lock = threading.Lock()
        self.state = None                 # 마지막 상태 dict
        self.state_time = 0.0
        self.alive = True
        threading.Thread(target=self._read_loop, daemon=True).start()

    def _connect(self, port):
        s = socket.create_connection((self.ip, port), timeout=3.0)
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return s

    # ── 명령 채널 ──────────────────────────────────────────────────────────
    def send(self, script, timeout=3.0):
        """스크립트 한 줄을 보내고 접수 응답을 기다린다 → (성공, 받은 글)."""
        with self.cmd_lock:
            self.cmd.settimeout(0.0)
            try:                                   # 앞에서 남은 응답 버리기
                while self.cmd.recv(4096):
                    pass
            except (BlockingIOError, socket.timeout):
                pass
            self.cmd.settimeout(timeout)
            self.cmd.sendall((script + "\n").encode())
            got, t_end = "", time.time() + timeout
            while ACK not in got and time.time() < t_end:
                try:
                    chunk = self.cmd.recv(4096)
                except socket.timeout:
                    break
                if not chunk:
                    raise ConnectionError("제어박스 명령 채널이 끊겼습니다")
                got += chunk.decode(errors="replace")
            return ACK in got, got.strip()

    # ── 데이터 채널 ────────────────────────────────────────────────────────
    def _read_loop(self):
        period = 1.0 / self.rate_hz
        while self.alive:
            t0 = time.time()
            try:
                st = self._request_state()
                if st is not None:
                    self.state, self.state_time = st, time.time()
            except (OSError, ConnectionError) as e:
                rospy.logerr_throttle(2.0, f"제어박스 데이터 채널 오류: {e}")
                time.sleep(0.5)
            time.sleep(max(0.0, period - (time.time() - t0)))

    def _request_state(self):
        self.data.settimeout(0.5)
        self.data.sendall(b"reqdata\n")
        buf = b""
        t_end = time.time() + 0.5
        while time.time() < t_end:
            chunk = self.data.recv(4096)
            if not chunk:
                raise ConnectionError("제어박스 데이터 채널이 끊겼습니다")
            buf += chunk
            i = buf.find(b"$")
            if i < 0:
                buf = b""
                continue
            buf = buf[i:]
            if len(buf) >= 4 and buf[3] != 3:      # 상태 묶음(종류 3)이 아니면 건너뛴다
                buf = buf[1:]
                continue
            if len(buf) >= STATE_MIN_BYTES:
                return self._parse(buf)
        return None

    @staticmethod
    def _parse(b):
        f = lambda off, n=1: struct.unpack_from(f"<{n}f", b, off)
        i = lambda off, n=1: struct.unpack_from(f"<{n}i", b, off)
        op = i(OFF_OP_STAT, 5)
        return {"jnt_ref_deg": f(OFF_JNT_REF, 6), "task_state": i(OFF_TASK_STATE)[0],
                "speed_bar": f(OFF_SPEED_BAR)[0], "robot_state": i(OFF_ROBOT_STATE)[0],
                "simulation": i(OFF_SIM_MODE)[0] == 1,
                "collision": op[0] != 0, "self_collision": op[2] != 0,
                "soft_estop": op[3] != 0, "emergency_stop": op[4] != 0,
                "inbox_trap": any(v != 0 for v in i(OFF_INBOX_TRAP, 2))}

    def close(self):
        self.alive = False
        for s in (self.cmd, self.data):
            try:
                s.close()
            except OSError:
                pass


class RbDriver:
    def __init__(self):
        self.fake = rospy.get_param("~fake", True)
        self.max_vel = float(rospy.get_param("~max_joint_velocity", 0.8))
        self.q = [0.0, -0.5236, 1.5708, 0.5236, 1.5708, 0.0]    # fake 시작 자세 = scene.yaml home
        self.box = None
        if not self.fake:
            ip = rospy.get_param("~robot_ip", "192.168.35.100")
            try:
                self.box = ControlBox(ip)
            except OSError as e:
                rospy.logfatal(f"제어박스 {ip} 에 접속할 수 없습니다 ({e}) — 랜선·IP·ping 을 확인하세요. "
                               "로봇 없이 하려면 mode:=sim")
                raise SystemExit(1)
            t0 = time.time()
            while self.box.state is None:
                if time.time() - t0 > 5.0:
                    rospy.logfatal("제어박스가 상태를 보내지 않습니다(데이터 채널 5001)")
                    raise SystemExit(1)
                time.sleep(0.05)
            sim = rospy.get_param("~cb_simulation", True)
            ok1, _ = self.box.send("pgmode " + ("simulation" if sim else "real"))
            ok2, _ = self.box.send(f"set_speed_bar({float(rospy.get_param('~speed_bar', 1.0)):.2f})")
            if not (ok1 and ok2):
                rospy.logwarn("제어박스가 모드·속도바 명령을 접수하지 않았습니다")
            rospy.loginfo(f"제어박스 {ip} 접속 — " + ("Simulation 모드(팔 안 움직임)" if sim else
                                                    "🔴 REAL 모드 — 팔이 실제로 움직입니다"))
        else:
            rospy.loginfo("가상 하드웨어(fake) — 궤적을 시간에 맞춰 흉내냅니다")

        self.js_pub = rospy.Publisher("arm/joint_states", JointState, queue_size=10)
        self.st_pub = rospy.Publisher("rb_driver/status", RobotStatus, queue_size=10)
        rospy.Service("rb_driver/script", RobotScript, self.on_script)
        self.server = actionlib.SimpleActionServer("arm_controller/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction, self.on_goal, auto_start=False)
        self.server.start()
        rospy.Timer(rospy.Duration(0.02), self.publish_joints)
        rospy.Timer(rospy.Duration(0.1), self.publish_status)

    # ── 상태 ─────────────────────────────────────────────────────────────
    def joints_now(self):
        if self.fake:
            return list(self.q)
        return [math.radians(v) for v in self.box.state["jnt_ref_deg"]]

    def publish_joints(self, _):
        if not self.fake and (self.box.state is None or time.time() - self.box.state_time > 1.0):
            rospy.logwarn_throttle(2.0, "제어박스 상태가 1초 넘게 안 들어옵니다 — 연결 확인")
            return
        m = JointState(name=JOINTS, position=self.joints_now())
        m.header.stamp = rospy.Time.now()
        self.js_pub.publish(m)

    def publish_status(self, _):
        m = RobotStatus()
        m.header.stamp = rospy.Time.now()
        if self.fake:
            m.connected, m.robot_state, m.simulation = True, IDLE, True
        elif self.box.state is not None:
            s = self.box.state
            m.connected = time.time() - self.box.state_time < 1.0
            m.robot_state, m.task_state, m.simulation, m.speed_bar = (s["robot_state"], s["task_state"],
                                                                      s["simulation"], s["speed_bar"])
            m.collision, m.self_collision = s["collision"], s["self_collision"]
            m.soft_estop, m.emergency_stop, m.inbox_trap = s["soft_estop"], s["emergency_stop"], s["inbox_trap"]
        self.st_pub.publish(m)

    def fault(self):
        """멈춰야 하는 제어박스 상태면 그 이유, 아니면 None."""
        s = self.box.state
        if time.time() - self.box.state_time > 1.0:
            return "제어박스 상태가 끊겼습니다"
        for k, why in (("emergency_stop", "비상정지"), ("soft_estop", "소프트 비상정지"), ("collision", "충돌 감지"),
                       ("self_collision", "자기 충돌"), ("inbox_trap", "인박스(안전 영역) 위반")):
            if s[k]:
                return why
        if s["robot_state"] == PAUSED:
            return "제어박스가 일시정지 상태입니다"
        return None

    # ── 스크립트 (그리퍼 등) ──────────────────────────────────────────────
    def on_script(self, req):
        if self.fake:
            rospy.loginfo(f"[fake] {req.script}")
            return RobotScriptResponse(success=True, message="fake")
        try:
            ok, got = self.box.send(req.script)
        except (OSError, ConnectionError) as e:
            return RobotScriptResponse(success=False, message=str(e))
        if not ok:
            rospy.logerr(f"제어박스가 명령을 접수하지 않음: {req.script} → {got}")
        return RobotScriptResponse(success=ok, message=got)

    # ── 궤적 실행 ────────────────────────────────────────────────────────
    def on_goal(self, goal):
        res = FollowJointTrajectoryResult()
        traj = goal.trajectory
        try:
            idx = [traj.joint_names.index(j) for j in JOINTS]
        except ValueError:
            res.error_code, res.error_string = res.INVALID_JOINTS, f"관절 이름이 {JOINTS} 와 다름"
            self.server.set_aborted(res, res.error_string)
            return
        t = [p.time_from_start.to_sec() for p in traj.points]
        q = [[p.positions[k] for k in idx] for p in traj.points]
        if len(q) < 2:
            self.server.set_succeeded(res)
            return
        why = self.check(t, q)
        if why:
            res.error_code, res.error_string = res.INVALID_GOAL, why
            rospy.logerr(f"궤적 거부: {why}")
            self.server.set_aborted(res, why)
            return
        done = self.run_fake(t, q) if self.fake else self.run_real(t, q, traj, idx)
        if done is True:
            self.server.set_succeeded(res)
        elif done == "preempted":
            self.server.set_preempted(res, "정지 요청으로 멈춤")
        else:
            res.error_code, res.error_string = res.PATH_TOLERANCE_VIOLATED, done
            rospy.logerr(f"궤적 실행 실패: {done}")
            self.server.set_aborted(res, done)

    def check(self, t, q):
        """실행 전 검사: 시간 순서 · 관절 속도 상한 · 시작 자세."""
        for a in range(1, len(t)):
            dt = t[a] - t[a - 1]
            if dt <= 0:
                return "궤적 시간이 늘어나지 않습니다"
            v = max(abs(q[a][j] - q[a - 1][j]) / dt for j in range(6))
            if v > self.max_vel * 1.05:
                return f"관절 속도 {v:.2f} rad/s > 상한 {self.max_vel:.2f} — 속도 배율을 낮추세요"
        start_err = max(abs(a - b) for a, b in zip(q[0], self.joints_now()))
        if start_err > 0.05:
            return f"궤적 시작 자세가 지금 자세와 {math.degrees(start_err):.1f}° 다릅니다"
        return None

    def run_fake(self, t, q):
        t0 = time.time()
        while True:
            if self.server.is_preempt_requested():
                return "preempted"
            now = time.time() - t0
            if now >= t[-1]:
                self.q = list(q[-1])
                return True
            k = max(i for i in range(len(t)) if t[i] <= now)
            s = (now - t[k]) / (t[k + 1] - t[k])
            self.q = [a + s * (b - a) for a, b in zip(q[k], q[k + 1])]
            time.sleep(0.01)

    def run_real(self, t, q, traj, idx):
        if self.box.state["robot_state"] == MOVING:
            return "로봇이 이미 움직이는 중입니다"
        why = self.fault()
        if why:
            return why
        vel, acc = self.derivatives(t, q, traj, idx)
        cmds = ["move_ros_j_clear()"]
        for k in range(len(t)):
            deg = lambda xs: ", ".join(f"{math.degrees(x):.3f}" for x in xs)
            cmds.append(f"move_ros_j_add({t[k]:.4f}, jnt[{deg(q[k])}], jnt[{deg(vel[k])}], jnt[{deg(acc[k])}])")
        cmds.append(f"move_ros_j_run(0,{len(t)})")
        for c in cmds:
            ok, got = self.box.send(c)
            if not ok:
                return f"제어박스가 궤적 명령을 접수하지 않음: {c[:40]}… → {got}"
        # 실행 감시: 움직이기 시작 → 정지 + 마지막 점 도착
        t_start, started = time.time(), False
        limit = t[-1] * 2.0 + 10.0
        while time.time() - t_start < limit:
            if self.server.is_preempt_requested():
                self.box.send("task stop")
                return "preempted"
            why = self.fault()
            if why:
                self.box.send("task stop")
                return why
            st = self.box.state["robot_state"]
            started |= st == MOVING
            err = max(abs(a - b) for a, b in zip(self.joints_now(), q[-1]))
            if st == IDLE and (started or time.time() - t_start > 1.0) and err < 0.01:
                return True
            if st == IDLE and started and err >= 0.01:
                return f"로봇이 멈췄지만 목표와 {math.degrees(err):.1f}° 차이 납니다"
            time.sleep(0.02)
        self.box.send("task stop")
        return f"{limit:.0f}초 안에 끝나지 않았습니다"

    @staticmethod
    def derivatives(t, q, traj, idx):
        """속도·가속도: 궤적에 있으면 그대로, 없으면 위치 차분으로 만든다(시작·끝은 0)."""
        n = len(t)
        has = all(len(p.velocities) == 6 and len(p.accelerations) == 6 for p in traj.points)
        if has:
            return ([[p.velocities[k] for k in idx] for p in traj.points],
                    [[p.accelerations[k] for k in idx] for p in traj.points])
        vel = [[0.0] * 6 for _ in range(n)]
        for k in range(1, n - 1):
            vel[k] = [(q[k + 1][j] - q[k - 1][j]) / (t[k + 1] - t[k - 1]) for j in range(6)]
        acc = [[0.0] * 6 for _ in range(n)]
        for k in range(1, n - 1):
            acc[k] = [(vel[k + 1][j] - vel[k - 1][j]) / (t[k + 1] - t[k - 1]) for j in range(6)]
        return vel, acc


def main():
    rospy.init_node("rb_driver")
    drv = RbDriver()
    rospy.on_shutdown(lambda: drv.box.close() if drv.box else None)
    rospy.spin()


if __name__ == "__main__":
    main()
