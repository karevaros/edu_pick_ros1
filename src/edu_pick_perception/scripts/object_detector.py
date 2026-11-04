#!/usr/bin/env python3
"""물체 인식 노드 — 위에서 내려다본 깊이 영상에서 테이블 위 물체를 찾는다.

■ 아이디어 (딥러닝 없이)
  카메라가 테이블을 수직으로 내려다보면, 테이블은 '거의 같은 높이의 평면'이다.
  ⇒ 테이블보다 높은 점들의 덩어리 = 물체.

  ① 점군(카메라 좌표) → TF 로 world(로봇 베이스) 좌표로 변환
  ② 테이블 높이 추정: 높이 히스토그램에서 가장 많은 높이 = 테이블 면
  ③ 테이블보다 min_height 이상 높은 점 = 물체 후보 (마스크)
     (손끝 tool_tip 보다 높은 점 = 카메라에 비친 그리퍼 손가락 → 뺀다)
  ④ 마스크에서 덩어리(윤곽선) 찾기 → 덩어리마다 3D 점을 모아
  ⑤ 윗면 점들에서 튀는 점을 뺀 뒤 '최소 외접 사각형'을 씌운다
       · 가로세로가 비슷하고 사각형을 덜 채우면(원은 78.5%) → round (캔·병·공)
       · 아니면 → box
       · 짧은 변 길이 = 집을 폭, 짧은 변 방향 = 그리퍼가 닫힐 방향(yaw)
  ⑥ 한 물체가 두 덩어리로 갈라진 것 합치기 (투명한 병은 깊이가 듬성듬성하다)
  ⑦ 결과 발행

  계산은 2D 영상 격자(덩어리 나누기)와 3D 점(치수) 을 섞어 쓴다 — 그래서 점군인데도 빠르다.
  점군은 가로·세로 decimate 칸마다 하나만 쓴다(기본 2 → 점 1/4).

■ 토픽
  입력  /camera/d405/depth/color/points      (sensor_msgs/PointCloud2, 격자형)
  출력  /edu_pick/objects                     (edu_pick_msgs/DetectedObjectArray)
        /edu_pick/objects_markers             (RViz 표시)
        /edu_pick/debug_image                 (단계별 결과 그림 — rqt_image_view 로 보기)
        /edu_pick/between_fingers             (std_msgs/Int32 — 손가락 사이 점 개수, 놓기 확인용)
        /edu_pick/detector_timing             (std_msgs/String JSON — 단계별 처리 시간 [ms])
"""
import json
import math
import time

import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Int32, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray

from edu_pick_msgs.msg import DetectedObject, DetectedObjectArray
from edu_pick_perception.common import default_scene_file, quat_to_mat, object_markers

# 손가락 사이 공간 (tool_tip 좌표 [m]) — 놓은 뒤 여기에 점이 남아 있으면 '아직 손에 있음'
#   |x|: 놓을 때 '물체 폭 + release_margin(3 cm)' 만 여므로 손가락이 ±1.5 cm 바깥에 온다
#   z: 손끝 위 0.3 ~ 3 cm (손끝 아래는 막 놓은 물체가 있으므로 보지 않는다)
BETWEEN_X, BETWEEN_Y, BETWEEN_Z = 0.015, 0.05, (-0.03, -0.003)
TALL_RATIO = 2.5          # 높이 ≥ 폭 × 2.5 = 키 큰 물체 (scene.yaml grasp_rules.tall_ratio 와 같은 값)


def cloud_to_arrays(msg, step=1):
    """PointCloud2(격자형) → xyz (H,W,3) float32, rgb (H,W,3) uint8.
    step > 1 이면 가로·세로 step 칸마다 하나만 쓴다(솎아내기 — 점 수 1/step²)."""
    fields = {f.name: f.offset for f in msg.fields}
    dtype = np.dtype({"names": ["x", "y", "z", "rgb"],
                      "formats": ["<f4", "<f4", "<f4", "<u4"],
                      "offsets": [fields["x"], fields["y"], fields["z"], fields["rgb"]],
                      "itemsize": msg.point_step})
    a = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height).reshape(msg.height, msg.width)
    if step > 1:
        a = a[::step, ::step]
    xyz = np.stack([a["x"], a["y"], a["z"]], axis=-1)
    c = a["rgb"]
    rgb = np.stack([(c >> 16) & 255, (c >> 8) & 255, c & 255], axis=-1).astype(np.uint8)
    return xyz, rgb


class ObjectDetector:
    def __init__(self):
        self.p_cloud_topic = rospy.get_param("~cloud_topic", "/camera/d405/depth/color/points")
        self.p_world_frame = rospy.get_param("~world_frame", "world")
        self.p_scene_file = rospy.get_param("~scene_file", default_scene_file())
        self.p_rate = rospy.get_param("~rate", 3.0)     # 초당 처리 횟수
        # 솎아내기: 2 = 가로·세로 2칸마다 1점(848×480 → 424×240, 점 1/4). 대기위치에서 점 간격 약 1.2 mm
        self.p_decimate = rospy.get_param("~decimate", 2)
        self._step = 1

        self.world = self.p_world_frame
        with open(self.p_scene_file) as f:
            scene = yaml.safe_load(f)
        self.cfg = scene["detection"]
        self.table_z_cfg = float(scene["table"]["top_z"])

        self.tf = Buffer()
        self._tfl = TransformListener(self.tf)
        self.bridge = CvBridge()
        self._last = None
        self._pending, self._retries = None, 0
        rospy.Subscriber(self.p_cloud_topic, PointCloud2, lambda m: setattr(self, "_last", m), queue_size=2)
        self.pub = rospy.Publisher("/edu_pick/objects", DetectedObjectArray, queue_size=10)
        self.pub_mk = rospy.Publisher("/edu_pick/objects_markers", MarkerArray, queue_size=10)
        self.pub_img = rospy.Publisher("/edu_pick/debug_image", Image, queue_size=2)
        # 손가락 사이에 보이는 점 개수 — 그리퍼를 연 상태에서 물체가 아직 손에 남아 있는지 확인용
        self.pub_between = rospy.Publisher("/edu_pick/between_fingers", Int32, queue_size=2)
        # 단계별 처리 시간(ms) — 프레임마다 JSON 으로 발행 (pick_place 가 기록·분석)
        self.pub_timing = rospy.Publisher("/edu_pick/detector_timing", String, queue_size=10)
        self.max_open = float(scene.get("gripper", {}).get("max_open", 0.10))
        rospy.Timer(rospy.Duration(1.0 / float(self.p_rate)), lambda _e: self.process())
        rospy.loginfo_throttle(5.0, f"물체 인식 시작 — 입력 {self.p_cloud_topic}")

    # ------------------------------------------------------------------ 처리
    def process(self):
        # TF 가 아직 안 온 프레임은 버리지 않고 다음 주기에 다시 처리한다 (아래 ① 참고)
        if self._pending is not None:
            msg, self._pending = self._pending, None
        else:
            msg, self._last = self._last, None
        if msg is None:
            return
        tm = {}                                     # 단계별 처리 시간 [ms]
        t0 = tk = time.perf_counter()

        def lap(name):
            nonlocal tk
            now = time.perf_counter()
            tm[name] = tm.get(name, 0.0) + (now - tk) * 1000.0
            tk = now
        if msg.height <= 1:
            rospy.logerr("점군이 격자형이 아닙니다 — 카메라를 pointcloud.ordered_pc:=true 로 띄우세요.",
                                    throttle_duration_sec=5.0)
            return
        # ① 카메라 → world 변환. **점군을 찍은 시각**의 TF 를 쓴다
        #    (가장 최근 TF 를 쓰면 팔이 움직이는 중에 찍힌 점군이 엉뚱한 곳에 놓인다)
        #    🔴 카메라 시각이 관절 TF 보다 수십 ms 앞서면 '미래 시각'이라 아직 계산할 수 없다.
        #       그런 프레임을 버리면 계속 결과가 안 나올 수 있다 → 보관했다가 다음 주기에 다시
        try:
            t = self.tf.lookup_transform(self.world, msg.header.frame_id, msg.header.stamp)
        except Exception as e:
            if "future" in str(e) and self._retries < 5:
                self._pending, self._retries = msg, self._retries + 1
                return
            self._retries = 0
            rospy.logwarn(f"TF 없음 {self.world}←{msg.header.frame_id}: {e}")
            return
        self._retries = 0
        step = max(1, int(self.p_decimate))
        self._step = step
        xyz_cam, rgb = cloud_to_arrays(msg, step)       # 메시지 해석은 한 번만
        lap("점군해석")
        self._count_between_fingers(msg, xyz_cam, step)
        lap("TF·손가락확인")
        R = quat_to_mat(t.transform.rotation)
        p = t.transform.translation
        H, W, _ = xyz_cam.shape
        valid = np.isfinite(xyz_cam[..., 2]) & (xyz_cam[..., 2] > 0.05)
        P = xyz_cam.reshape(-1, 3) @ R.T + np.array([p.x, p.y, p.z])
        P = P.reshape(H, W, 3)
        X, Y, Z = P[..., 0], P[..., 1], P[..., 2]

        c = self.cfg
        roi = valid & (X > c["roi_x"][0]) & (X < c["roi_x"][1]) & (Y > c["roi_y"][0]) & (Y < c["roi_y"][1])
        lap("좌표변환")
        if roi.sum() < 500:
            self._publish([], msg.header.stamp, rgb, None)
            return

        # ② 테이블 높이: ROI 안 높이 히스토그램의 최빈값(4 mm 간격)
        table_z = self.table_z_cfg
        if c.get("auto_table", True):
            zs = Z[roi]
            zs = zs[np.abs(zs - self.table_z_cfg) < 0.10]
            if zs.size > 500:
                hist, edges = np.histogram(zs, bins=np.arange(self.table_z_cfg - 0.10, self.table_z_cfg + 0.10, 0.004))
                k = int(np.argmax(hist))
                if hist[k] > 0.15 * zs.size:
                    table_z = float(0.5 * (edges[k] + edges[k + 1]))
        lap("테이블면추정")

        # ③ 물체 마스크
        hgt = Z - table_z
        # 손가락 제외: 위에서 내려다보므로 물체는 항상 손끝(tool_tip)보다 아래에 있다.
        #   손끝보다 1.5 cm 위의 점은 그리퍼 자신(손가락이 카메라 시야에 들어온다)
        z_max = np.inf
        try:
            tip = self.tf.lookup_transform(self.world, "tool_tip", msg.header.stamp)
            z_max = tip.transform.translation.z - 0.015
        except Exception:
            pass
        mask = (roi & (hgt > c["min_height"]) & (hgt < c["max_height"]) & (Z < z_max)).astype(np.uint8) * 255
        k5 = np.ones((5, 5) if step == 1 else (3, 3), np.uint8)     # 솎아낸 만큼 작은 커널
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k5)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5)

        # ④ 덩어리마다 ⑤ 모양·크기·방향
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lap("물체분리")
        objects = []
        for cnt in contours:
            if cv2.contourArea(cnt) < 100 / step ** 2:
                continue
            blob = np.zeros((H, W), np.uint8)
            cv2.drawContours(blob, [cnt], -1, 255, -1)
            # 깊이 카메라는 물체 경계에 '튀는 점'(앞뒤 거리가 섞인 점)을 만든다 → 테두리 2 px 를 깎고,
            # 그래도 남은 먼 점은 중심 거리로 버린다 (1~2 % 의 점만으로 사과가 29 cm 로 부푼 적이 있다)
            core = cv2.erode(blob, k5)
            # valid: 마스크를 메우는 연산(CLOSE)이 깊이 없는(NaN) 픽셀을 끌어들일 수 있다 → 유효한 점만
            sel = (core > 0) & (mask > 0) & valid
            pts = P[sel]
            if len(pts) < max(15, 50 // step ** 2):
                continue
            d = np.linalg.norm(pts[:, :2] - np.median(pts[:, :2], axis=0), axis=1)
            pts = pts[d <= 1.5 * np.percentile(d, 90)]
            if len(pts) < 15:
                continue
            lap("물체분리")                          # 덩어리별 점 추출까지 '분리'
            o = self.describe(pts, table_z)
            lap("물체판별")                          # 모양·크기·방향·위치 = 이 실습의 '추론'에 해당
            if o is None:
                continue
            # 시야(영상 테두리)나 인식 영역(roi) 경계에 걸린 덩어리는 일부만 보인 것 → 크기를 믿을 수 없다
            x, y, w, h = cv2.boundingRect(cnt)
            at_image_edge = x <= 2 or y <= 2 or x + w >= W - 2 or y + h >= H - 2
            xs, ys = pts[:, 0], pts[:, 1]
            at_roi_edge = (xs.min() < c["roi_x"][0] + 0.005 or xs.max() > c["roi_x"][1] - 0.005 or
                           ys.min() < c["roi_y"][0] + 0.005 or ys.max() > c["roi_y"][1] - 0.005)
            if (at_image_edge or at_roi_edge) and o.graspable:   # 이미 '너무 넓음'이면 그 이유가 우선
                o.graspable = False
                o.note = "가장자리에 잘림"
            if o.width < float(c.get("min_width", 0.0)):     # 케이블처럼 가는 것은 물체가 아니다
                continue
            objects.append((o, cnt))

        # 한 물체가 여러 덩어리로 갈라진 것 합치기: 투명 PET 몸통은 깊이가 듬성듬성해서
        #   세운 병이 '뚜껑'과 '몸통 조각'으로 나뉜다(그대로 두면 몸통 조각을 집으려다 뚜껑 덩어리에 막힌다).
        #   중심이 다른(더 높은) 덩어리의 외곽 반경 안이면 같은 물체 → 더 높은 덩어리 하나로 남긴다
        objects.sort(key=lambda oc: -oc[0].height)
        merged = []
        for o, cnt in objects:
            host = next((h for h in merged if math.hypot(o.top_center.x - h[0].top_center.x,
                                                         o.top_center.y - h[0].top_center.y)
                         < 0.5 * max(h[0].width, o.width) + 0.01), None)   # 폭 기준(길이면 누운 병 옆 물체까지 삼킨다)
            if host is None:
                merged.append((o, cnt))
        objects = merged
        for i, (o, _) in enumerate(objects):
            o.id = f"obj_{i}"

        self._publish(objects, msg.header.stamp, rgb, mask, table_z)
        lap("결과발행")
        tm["합계"] = (time.perf_counter() - t0) * 1000.0
        stamp = msg.header.stamp.to_sec()
        tm["카메라→처리완료"] = (rospy.Time.now().to_sec() - stamp) * 1000.0
        self.pub_timing.publish(String(data=json.dumps(
            {"stamp": stamp, "objects": len(objects), "ms": {k: round(v, 2) for k, v in tm.items()}},
            ensure_ascii=False)))

    def _count_between_fingers(self, msg, xyz_cam, step):
        """손끝(tool_tip) 좌표에서 두 손가락 사이 공간(BETWEEN_*)의 점 개수 = 손에 남은 물체.
        tool_tip: z = 접근(아래) 방향 · x = 손가락이 닫히는 방향"""
        try:
            t = self.tf.lookup_transform("tool_tip", msg.header.frame_id, msg.header.stamp)
        except Exception:
            return
        # 손가락·손에 든 물체는 카메라에서 30 cm 안에 있다 → 그 점만 변환한다(전체 변환은 낭비)
        P = xyz_cam.reshape(-1, 3)
        P = P[np.isfinite(P[:, 2]) & (P[:, 2] > 0.02) & (P[:, 2] < 0.30)]
        p = t.transform.translation
        P = P @ quat_to_mat(t.transform.rotation).T + np.array([p.x, p.y, p.z])
        n = int(np.count_nonzero((np.abs(P[:, 0]) < BETWEEN_X) & (np.abs(P[:, 1]) < BETWEEN_Y) &
                                 (P[:, 2] > BETWEEN_Z[0]) & (P[:, 2] < BETWEEN_Z[1]))) * step * step  # 솎아내기 전 점 수로 환산
        self.pub_between.publish(Int32(data=n))

    @staticmethod
    def robust_box(xy, lo=2.0, hi=98.0, pad=0.10):
        """튀는 점을 뺀 외접 상자.
        ① 주성분(PCA)으로 긴 축·짧은 축을 잡고, 각 축에서 하위 2 %~상위 98 % 범위를 구한다
        ② 그 범위를 양쪽으로 10 % 넓힌 창 밖의 점 = 튀는 점 → 버린다
        ③ 남은 점에 최소 외접 사각형(cv2.minAreaRect)
        ⚠ ①의 범위를 그대로 크기로 쓰면 원(디스크)은 가장자리 점이 적어 15~20 % 작게 나온다(가상 시험에서 확인).
          ③을 전체 점에 바로 쓰면 튀는 점 몇 개에 크게 부푼다(예: 누운 페트병 5.8 → 11.6 cm).
        반환: 중심(x, y), 폭(짧은 변), 길이(긴 변), 짧은 변 방향 단위벡터, 볼록 껍질 면적"""
        m = xy.mean(axis=0)
        d = xy - m
        _, vecs = np.linalg.eigh(np.cov(d.T))
        keep = np.ones(len(xy), bool)
        for axis in (vecs[:, 0], vecs[:, 1]):
            t = d @ axis
            t0, t1 = np.percentile(t, [lo, hi])
            margin = pad * (t1 - t0)
            keep &= (t >= t0 - margin) & (t <= t1 + margin)
        pts = xy[keep].astype(np.float32)
        hull = cv2.convexHull(pts)
        rect = cv2.minAreaRect(hull)
        (cx, cy), (a, b), _ = rect
        box = cv2.boxPoints(rect)
        e1, e2 = box[1] - box[0], box[2] - box[1]
        short = e1 if np.linalg.norm(e1) <= np.linalg.norm(e2) else e2
        short = short / max(np.linalg.norm(short), 1e-9)
        return (cx, cy), float(min(a, b)), float(max(a, b)), short, float(cv2.contourArea(hull))

    def describe(self, pts, table_z):
        """한 덩어리의 3D 점 → DetectedObject (모양·중심·폭·방향·높이).

        ⚠ 카메라가 물체 바로 위가 아니면 옆면도 보인다 → 전체 외곽은 부푼다.
          그래서 모양·중심·방향은 **윗면 띠**(윗면에서 1.5 cm 이내 점)로 정하고,
          전체 외곽 폭은 '그리퍼에 들어가나' 판정에만 쓴다.
        크기는 튀는 점에 강한 robust_box 로 잰다."""
        c = self.cfg
        h = pts[:, 2] - table_z
        top = float(np.percentile(h, 95))           # 윗면 높이 (튀는 점 5% 무시)

        _, outer, _, _, area_all = self.robust_box(pts[:, :2])
        if area_all < c["min_area"]:                # 위에서 본 면적 [m²]이 너무 작으면 잡음
            return None

        # 윗면 띠: 보통 1.5 cm. 키 큰 물체(세운 병)는 띠에 뚜껑 옆면이 섞여 중심이 카메라 쪽으로 끌린다
        #   → 0.8 cm 로 좁힌다 (넓으면 그리퍼가 뚜껑 옆으로 빗나간다)
        band = 0.015
        if top >= TALL_RATIO * max(outer, 1e-3):
            band = 0.008
        top_pts = pts[h > top - band]
        if len(top_pts) < 20:
            return None
        (cx, cy), width, length, short_dir, area = self.robust_box(top_pts[:, :2])
        if width > outer + 0.005:                   # 윗면은 외곽보다 넓을 수 없다 → 잡음, 외곽으로
            (cx, cy), width, length, short_dir, area = self.robust_box(pts[:, :2])

        fill = area / max(width * length, 1e-9)     # 사각형을 얼마나 채우나 (원 ≈ 0.785)
        aspect = length / max(width, 1e-9)
        o = DetectedObject()
        if aspect < 1.2 and fill < 0.88:
            o.shape = "round"
            length = width = (width + length) / 2
            o.yaw = float("nan")                     # 원은 어느 방향으로 잡아도 같다
        else:
            o.shape = "box"
            yaw = math.atan2(float(short_dir[1]), float(short_dir[0]))
            # 그리퍼는 180° 대칭 → -90°~+90° 로 정리
            while yaw > math.pi / 2:
                yaw -= math.pi
            while yaw <= -math.pi / 2:
                yaw += math.pi
            o.yaw = yaw
        o.top_center = Point(x=float(cx), y=float(cy), z=table_z + top)
        o.width, o.length, o.height = width, max(length, width), top
        # 집을 수 있나: 윗면 폭 ≤ max_grip_width, 그리고 옆면까지 본 외곽 폭 ≤ 그리퍼 최대 벌림
        #   (외곽 폭은 비스듬히 보인 옆면이 섞여 1~2 cm 크게 나온다 — 예: 사과 윗면 8.2 · 외곽 9.7 cm)
        o.graspable = width <= float(c["max_grip_width"]) and outer <= self.max_open
        if not o.graspable:
            o.note = (f"너무 넓음(윗면 {width * 100:.1f} cm > {float(c['max_grip_width']) * 100:.1f} cm)"
                      if width > float(c["max_grip_width"]) else
                      f"너무 넓음(외곽 {outer * 100:.1f} cm > 벌림 {self.max_open * 100:.0f} cm)")
        return o

    # ------------------------------------------------------------------ 발행
    def _publish(self, objects, stamp, rgb, mask, table_z=None):
        """table_z=None: 테이블을 못 찾은 프레임(물체 0개 · 그림에 높이 표시 안 함)"""
        out = DetectedObjectArray()
        out.header.stamp = stamp
        out.header.frame_id = self.world
        out.objects = [o for o, _ in objects]
        self.pub.publish(out)
        self.pub_mk.publish(object_markers(out.objects, self.world))

        # 디버그 그림: 컬러 영상 + 물체 마스크(초록) + 윤곽·이름 — 보는 사람이 있을 때만 그린다
        if self.pub_img.get_num_connections() == 0:
            return
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if mask is not None:
            img[mask > 0] = (0.5 * img[mask > 0] + [0, 110, 0]).astype(np.uint8)
        # 솎아낸 영상은 작다 → 원래 크기로 키운 뒤 글자를 쓴다(작은 영상에 쓰면 글자가 겹친다)
        k = self._step
        if k > 1:
            img = cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_NEAREST)
        for o, cnt in objects:
            col = (0, 255, 0) if o.graspable else (0, 0, 255)
            cnt = cnt * k
            cv2.drawContours(img, [cnt], -1, col, 2)
            x, y, _, _ = cv2.boundingRect(cnt)
            label = f"{o.id} {o.shape} {o.width * 100:.1f}cm" + ("" if o.graspable else " X")
            cv2.putText(img, label, (x, max(15, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
        if table_z is not None:
            cv2.putText(img, f"table z = {table_z:.3f} m   objects = {len(objects)}", (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        self.pub_img.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))


def main():
    rospy.init_node("object_detector")
    ObjectDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
