#!/usr/bin/env python3
"""가상 손목 카메라 + 가상 물체 — 로봇·카메라 없이(mode:=sim) 실습 전체를 돌려 보기 위한 노드.

■ 하는 일
  1) scene.yaml 의 sim_objects(박스·원기둥)를 테이블 위에 둔다.
  2) 손목 카메라(d405_link)의 현재 위치에서 보이는 장면을 **광선 추적**으로 그려
     실제 D405 드라이버와 같은 모양의 점군을 발행한다.
        /camera/d405/depth/color/points   (격자형 PointCloud2, 프레임 d405_depth_optical_frame)
     ⇒ 실기와 **똑같은 object_detector 가 그대로** 이 점군을 처리한다.
  3) 그리퍼가 닫힐 때(rg2_finger_joint1 이 작아질 때) 손끝 사이에 물체가 있으면 '집힌다'
     — 그 뒤로 물체는 손끝을 따라 움직이고, 그리퍼가 열리면 그 자리 테이블 위에 놓인다.
     박스는 그리퍼 방향이 짧은 변과 20° 넘게 어긋나면 미끄러져 못 집는다(경고 로그).

■ 한계 — 물리 시뮬레이션이 아니다
  힘·마찰·미끄러짐은 계산하지 않는다. '닫힐 때 손끝 사이에 있었나'만 본다.
"""
import math

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState, PointCloud2, PointField
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from edu_pick_perception.common import default_scene_file, quat_to_mat

# D405 시야각(가로 87°, 세로 58°) · 해상도는 계산량 때문에 실제(848×480)의 절반
W, H = 424, 240
FX = (W / 2) / math.tan(math.radians(87 / 2))
FY = (H / 2) / math.tan(math.radians(58 / 2))
OPTICAL = "d405_depth_optical_frame"
COLORS = {"table": (190, 155, 115), "box": (40, 90, 220), "round": (220, 60, 40)}


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class FakeCamera:
    def __init__(self):
        self.p_scene_file = rospy.get_param("~scene_file", default_scene_file())
        self.p_rate = rospy.get_param("~rate", 5.0)
        self.p_noise = rospy.get_param("~noise", 0.001)     # 깊이 잡음 표준편차 [m]
        with open(self.p_scene_file) as f:
            scene = yaml.safe_load(f)
        self.table_z = float(scene["table"]["top_z"])
        g = scene["gripper"]
        self.max_open = float(g["max_open"])                          # 최대 벌림 [m]
        self.tip_advance = float(g["tip_closed"]) - float(g["tip_open"])   # 닫히며 손끝이 나오는 거리 [m]
        tc, ts = scene["table"]["center_xy"], scene["table"]["size_xy"]
        self.table_box = (tc[0] - ts[0] / 2, tc[0] + ts[0] / 2, tc[1] - ts[1] / 2, tc[1] + ts[1] / 2)

        self.objs = []
        for o in scene.get("sim_objects", []):
            d = {"id": o["id"], "shape": o["shape"], "x": float(o["xy"][0]), "y": float(o["xy"][1]),
                 "yaw": math.radians(float(o.get("yaw_deg", 0.0))), "z0": self.table_z}
            if o["shape"] == "box":
                d["sx"], d["sy"], d["h"] = (float(v) for v in o["size"])
            else:
                d["r"], d["h"] = float(o["size"][0]) / 2, float(o["size"][1])
            self.objs.append(d)
        self.held = None          # (물체, 손끝 기준 위치 오프셋, 손끝 기준 방향 오프셋)
        self.finger = None

        self.tf = Buffer()
        self._tfl = TransformListener(self.tf)
        # 실제 드라이버처럼 카메라 몸체 → 광학 프레임 TF 를 낸다 (z 앞, x 오른쪽, y 아래)
        st = TransformStamped()
        st.header.frame_id, st.child_frame_id = "d405_link", OPTICAL
        st.transform.rotation.x, st.transform.rotation.y = -0.5, 0.5
        st.transform.rotation.z, st.transform.rotation.w = -0.5, 0.5
        self._stf = StaticTransformBroadcaster()
        self._stf.sendTransform(st)

        self.pub = rospy.Publisher("/camera/d405/depth/color/points", PointCloud2, queue_size=2)
        self.pub_mk = rospy.Publisher("/edu_pick/sim_objects", MarkerArray, queue_size=2)
        uu, vv = np.meshgrid(np.arange(W), np.arange(H))
        self.rays = np.stack([(uu - W / 2) / FX, (vv - H / 2) / FY, np.ones_like(uu, float)], -1).reshape(-1, 3)
        self.rng = np.random.default_rng(0)
        # rospy 는 만드는 즉시 콜백이 돌기 시작한다 → 준비가 다 끝난 뒤에 만든다
        rospy.Subscriber("/joint_states", JointState, self._on_js, queue_size=10)
        rospy.Timer(rospy.Duration(1.0 / float(self.p_rate)), lambda _e: self._render())
        rospy.Timer(rospy.Duration(0.1), lambda _e: self._markers())
        rospy.loginfo(f"가상 카메라 시작 — 가상 물체 {len(self.objs)}개")

    # ------------------------------------------------------------------ TF
    def _pose(self, frame, with_stamp=False):
        try:
            t = self.tf.lookup_transform("world", frame, rospy.Time(0))
        except Exception:
            return (None, None, None) if with_stamp else (None, None)
        p = t.transform.translation
        out = (np.array([p.x, p.y, p.z]), quat_to_mat(t.transform.rotation))
        return out + (t.header.stamp,) if with_stamp else out

    def _obj_pose(self, o):
        """들려 있으면 손끝을 따라간다."""
        if self.held is not None and self.held[0] is o:
            p, R = self._pose("tool_tip")
            if p is not None:
                off, dyaw = self.held[1], self.held[2]
                tip_yaw = math.atan2(R[1, 0], R[0, 0])
                return p[0] + off[0], p[1] + off[1], p[2] + off[2], tip_yaw + dyaw
        return o["x"], o["y"], o["z0"], o["yaw"]

    # ------------------------------------------------------------------ 집기 흉내
    def _on_js(self, msg):
        if "rg2_finger_joint1" not in msg.name:
            return
        q = msg.position[msg.name.index("rg2_finger_joint1")]
        prev, self.finger = self.finger, q
        if prev is None:
            return
        # 닫히는 방향으로 움직이면 '쥐기', 열리는 방향이면 '놓기' (닫기 목표가 물체마다 달라 고정 문턱은 안 된다)
        if q < prev - 0.05 and self.held is None:
            self._try_grasp()
        elif q > prev + 0.05 and self.held is not None:
            self._release()

    def _try_grasp(self):
        p, R = self._pose("tool_tip")
        if p is None:
            return
        close_dir = R[:, 0]               # tool_tip x축 = 손가락이 닫히는 방향
        for o in self.objs:
            dx, dy = p[0] - o["x"], p[1] - o["y"]
            top = o["z0"] + o["h"]
            half = o.get("r", None) or min(o["sx"], o["sy"]) / 2
            # RG2 는 닫을수록 손끝이 앞(아래)으로 나온다 — 실측 20.5 cm(10 cm 벌림) → 24 cm(닫힘), 직선 근사
            advance = self.tip_advance * (1.0 - min(2 * half / self.max_open, 1.0))
            tip_closed = p[2] - advance
            if math.hypot(dx, dy) > max(0.02, half * 0.6) or not (o["z0"] - 0.01 < tip_closed < top - 0.005):
                continue
            if o["shape"] == "box":
                short = o["yaw"] if o["sx"] <= o["sy"] else o["yaw"] + math.pi / 2
                ang = abs(math.atan2(math.sin(2 * (math.atan2(close_dir[1], close_dir[0]) - short)),
                                     math.cos(2 * (math.atan2(close_dir[1], close_dir[0]) - short)))) / 2
                if math.degrees(ang) > 20:
                    rospy.logwarn(f"[가상] {o['id']}: 그리퍼 방향이 짧은 변과 {math.degrees(ang):.0f}° 어긋나 미끄러짐")
                    return
                if min(o["sx"], o["sy"]) > self.max_open:
                    rospy.logwarn(f"[가상] {o['id']}: 너무 커서 손가락 사이에 안 들어감")
                    return
            tip_yaw = math.atan2(R[1, 0], R[0, 0])
            self.held = (o, (o["x"] - p[0], o["y"] - p[1], o["z0"] - p[2]), o["yaw"] - tip_yaw)
            rospy.loginfo(f"[가상] {o['id']} 집힘")
            return
        rospy.logwarn("[가상] 그리퍼를 닫았지만 손끝 사이에 물체가 없음")

    def _release(self):
        o = self.held[0]
        x, y, z, yaw = self._obj_pose(o)
        self.held = None
        drop = z - self.table_z
        o.update(x=x, y=y, z0=self.table_z, yaw=yaw)
        rospy.loginfo(f"[가상] {o['id']} 놓음 — 테이블 위 {drop * 100:.1f} cm 에서"
                               + (" (⚠ 높이서 떨어뜨림)" if drop > 0.03 else ""))

    # ------------------------------------------------------------------ 광선 추적
    def _render(self):
        cam, R, stamp = self._pose(OPTICAL, with_stamp=True)
        if cam is None:
            return
        d = self.rays @ R.T                         # 월드 좌표 광선 방향 (광학 z 성분 = 1)
        t_best = np.full(len(d), np.inf)
        col = np.zeros((len(d), 3), np.uint8)

        def hit(t, c):
            m = t < t_best
            t_best[m] = t[m]
            col[m] = c

        with np.errstate(divide="ignore", invalid="ignore"):
            # 테이블 윗면
            t = (self.table_z - cam[2]) / d[:, 2]
            px, py = cam[0] + t * d[:, 0], cam[1] + t * d[:, 1]
            x0, x1, y0, y1 = self.table_box
            t[~((t > 0) & (px > x0) & (px < x1) & (py > y0) & (py < y1))] = np.inf
            hit(t, COLORS["table"])
            for o in self.objs:
                ox, oy, oz, oyaw = self._obj_pose(o)
                Rz = _rot_z(-oyaw)
                lo = (cam - [ox, oy, oz]) @ Rz.T
                ld = d @ Rz.T
                if o["shape"] == "box":
                    hx, hy = o["sx"] / 2, o["sy"] / 2
                    t1 = (np.array([-hx, -hy, 0]) - lo) / ld
                    t2 = (np.array([hx, hy, o["h"]]) - lo) / ld
                    tmin = np.nanmax(np.minimum(t1, t2), axis=1)
                    tmax = np.nanmin(np.maximum(t1, t2), axis=1)
                    t = np.where((tmax >= tmin) & (tmin > 0), tmin, np.inf)
                else:
                    r, hgt = o["r"], o["h"]
                    a = ld[:, 0] ** 2 + ld[:, 1] ** 2
                    b = 2 * (lo[0] * ld[:, 0] + lo[1] * ld[:, 1])
                    cc = lo[0] ** 2 + lo[1] ** 2 - r * r
                    disc = b * b - 4 * a * cc
                    ts = (-b - np.sqrt(disc)) / (2 * a)
                    zs = lo[2] + ts * ld[:, 2]
                    t_side = np.where((disc >= 0) & (ts > 0) & (zs >= 0) & (zs <= hgt), ts, np.inf)
                    tc = (hgt - lo[2]) / ld[:, 2]
                    cx, cy = lo[0] + tc * ld[:, 0], lo[1] + tc * ld[:, 1]
                    t_cap = np.where((tc > 0) & (cx * cx + cy * cy <= r * r), tc, np.inf)
                    t = np.minimum(t_side, t_cap)
                hit(t, COLORS[o["shape"]])

        ok = np.isfinite(t_best) & (t_best > 0.07) & (t_best < 1.5)   # D405 측정 범위
        z = t_best + self.rng.normal(0, float(self.p_noise), len(t_best))
        xyz = self.rays * z[:, None]
        xyz[~ok] = np.nan
        self.pub.publish(self._cloud(xyz.astype(np.float32), col, stamp))

    def _cloud(self, xyz, col, stamp):
        msg = PointCloud2()
        msg.header.stamp = stamp      # 그린 시점의 카메라 자세와 같은 시각 → 인식 노드가 정확히 되돌릴 수 있다
        msg.header.frame_id = OPTICAL
        msg.height, msg.width = H, W
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32 if n != "rgb" else PointField.UINT32,
                                 count=1) for i, n in enumerate(["x", "y", "z", "rgb"])]
        msg.point_step, msg.row_step = 16, 16 * W
        msg.is_dense, msg.is_bigendian = False, False
        rgb = (col[:, 0].astype(np.uint32) << 16) | (col[:, 1].astype(np.uint32) << 8) | col[:, 2]
        buf = np.empty(len(xyz), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
        buf["x"], buf["y"], buf["z"], buf["rgb"] = xyz[:, 0], xyz[:, 1], xyz[:, 2], rgb
        msg.data = buf.tobytes()
        return msg

    # ------------------------------------------------------------------ RViz 표시
    def _markers(self):
        ma = MarkerArray()
        for i, o in enumerate(self.objs):
            x, y, z, yaw = self._obj_pose(o)
            m = Marker()
            m.header.frame_id = "world"
            m.ns, m.id = "sim_objects", i
            m.type = Marker.CUBE if o["shape"] == "box" else Marker.CYLINDER
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z + o["h"] / 2
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            if o["shape"] == "box":
                m.scale.x, m.scale.y = o["sx"], o["sy"]
            else:
                m.scale.x = m.scale.y = 2 * o["r"]
            m.scale.z = o["h"]
            c = COLORS[o["shape"]]
            m.color.r, m.color.g, m.color.b, m.color.a = c[0] / 255, c[1] / 255, c[2] / 255, 1.0
            ma.markers.append(m)
        self.pub_mk.publish(ma)


def main():
    rospy.init_node("fake_camera")
    FakeCamera()
    rospy.spin()


if __name__ == "__main__":
    main()
