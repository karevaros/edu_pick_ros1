#!/usr/bin/env python3
"""가짜 제어박스 — 로봇 없이 rb_driver 의 '실기 경로'(mode:=real)를 시험하는 도구.

    python3 tools/mock_controlbox.py                 # 127.0.0.1 의 5000·5001 에서 기다린다
    roslaunch edu_pick_bringup pick_place.launch mode:=real robot_ip:=127.0.0.1 camera:=false

제어박스와 같은 규약으로 답한다:
  명령 채널 5000  한 줄 받으면 "The command was executed" 응답
                  move_ros_j_clear / move_ros_j_add / move_ros_j_run → 궤적을 시간에 맞춰 '실행'
                  task stop → 멈춤 · pgmode · set_speed_bar · gripper_macro → 기록만
  데이터 채널 5001 "reqdata" 받으면 '$' 상태 묶음(관절 기준값·로봇 상태 …)

  --fault inbox|collision|pause   궤적 실행 중 절반 시점에 그 결함을 낸다(드라이버 감시 시험)
⚠ 시험 도구일 뿐 — 실제 제어박스의 동작(가감속·안전 기능)을 흉내내지 않는다.
"""
import argparse
import re
import socket
import struct
import threading
import time

HOME_DEG = [0.0, -30.0, 90.0, 30.0, 90.0, 0.0]
SIZE = 572


class MockBox:
    def __init__(self, fault=None):
        self.q = list(HOME_DEG)
        self.robot_state = 1
        self.traj = []
        self.run = None                     # (시작 시각, 점들)
        self.fault, self.fault_now = fault, None
        self.lock = threading.Lock()
        self.log = []

    def state_packet(self):
        b = bytearray(SIZE)
        b[0:4] = b"$" + struct.pack("<H", SIZE - 4) + b"\x03"
        with self.lock:
            self._advance()
            struct.pack_into("<6f", b, 8, *self.q)
            struct.pack_into("<i", b, 340, self.robot_state)
            struct.pack_into("<f", b, 336, 1.0)
            struct.pack_into("<i", b, 404, 1)
            if self.fault_now == "collision":
                struct.pack_into("<i", b, 444, 1)
            if self.fault_now == "inbox":
                struct.pack_into("<i", b, 472, 1)
        return bytes(b)

    def _advance(self):
        if self.run is None:
            return
        t0, pts = self.run
        now = time.time() - t0
        if now < 0:                         # 아직 출발 전
            return
        if self.fault and self.fault_now is None and now > pts[-1][0] / 2:
            self.fault_now = self.fault
            self.robot_state = 2
            self.run = None
            return
        if now >= pts[-1][0]:
            self.q = list(pts[-1][1])
            self.run, self.robot_state = None, 1
            return
        k = max(i for i in range(len(pts)) if pts[i][0] <= now)
        s = (now - pts[k][0]) / (pts[k + 1][0] - pts[k][0])
        self.q = [a + s * (c - a) for a, c in zip(pts[k][1], pts[k + 1][1])]
        self.robot_state = 3

    def command(self, line):
        self.log.append(line)
        with self.lock:
            if line.startswith("move_ros_j_clear"):
                self.traj = []
            elif line.startswith("move_ros_j_add"):
                nums = [float(x) for x in re.findall(r"-?\d+\.?\d*", line)]
                self.traj.append((nums[0], nums[1:7]))
            elif line.startswith("move_ros_j_run"):
                if len(self.traj) >= 2:
                    self.run = (time.time() + 0.15, list(self.traj))   # 실제처럼 조금 늦게 출발
            elif line.startswith("task stop"):
                self.run, self.robot_state = None, 1
        print("cmd:", line[:90], flush=True)


def serve(port, handler):
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(4)
    while True:
        c, _ = srv.accept()
        threading.Thread(target=handler, args=(c,), daemon=True).start()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fault", choices=["inbox", "collision", "pause"])
    box = MockBox(ap.parse_args().fault)

    def cmd_handler(c):
        buf = b""
        while True:
            d = c.recv(65536)
            if not d:
                return
            buf += d
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                box.command(line.decode().strip())
                c.sendall(b"The command was executed\n")

    def data_handler(c):
        while True:
            d = c.recv(64)
            if not d:
                return
            if b"reqdata" in d:
                c.sendall(box.state_packet())

    threading.Thread(target=serve, args=(5000, cmd_handler), daemon=True).start()
    threading.Thread(target=serve, args=(5001, data_handler), daemon=True).start()
    print("가짜 제어박스 대기 중 — 127.0.0.1:5000·5001", flush=True)
    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
