"""인식 노드들이 함께 쓰는 작은 함수들."""
import math
import os

import numpy as np
import rospkg
from visualization_msgs.msg import Marker, MarkerArray


def default_scene_file():
    return os.path.join(rospkg.RosPack().get_path("edu_pick_description"), "config", "scene.yaml")


def quat_to_mat(q):
    """geometry_msgs Quaternion → 3×3 회전행렬."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def object_markers(objects, frame, ns="detected"):
    """DetectedObject 목록 → RViz 마커(초록 = 집을 수 있음 / 빨강 = 너무 큼)."""
    ma = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    ma.markers.append(clear)
    for i, o in enumerate(objects):
        m = Marker()
        m.header.frame_id = frame
        m.ns, m.id = ns, i
        m.type = Marker.CYLINDER if o.shape == "round" else Marker.CUBE
        m.pose.position.x = o.top_center.x
        m.pose.position.y = o.top_center.y
        m.pose.position.z = o.top_center.z - o.height / 2
        yaw = 0.0 if math.isnan(o.yaw) else o.yaw
        m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.scale.x, m.scale.y, m.scale.z = o.width, o.length, max(o.height, 0.005)
        if o.shape == "round":
            m.scale.y = o.width
        m.color.a = 0.45
        m.color.r, m.color.g = (0.1, 0.9) if o.graspable else (0.9, 0.1)
        ma.markers.append(m)

        t = Marker()
        t.header.frame_id = frame
        t.ns, t.id, t.type = ns + "_label", i, Marker.TEXT_VIEW_FACING
        t.pose.position.x, t.pose.position.y = o.top_center.x, o.top_center.y
        t.pose.position.z = o.top_center.z + 0.04
        t.scale.z = 0.025
        t.color.r = t.color.g = t.color.b = t.color.a = 1.0
        # ⚠ 공백을 넣으면 RViz 가 단어 사이를 크게 벌려 그려 라벨이 흩어진다 → 공백 없이
        t.text = f"{o.id}/{o.shape}/{o.width * 100:.1f}cm" + ("" if o.graspable else "/X")
        ma.markers.append(t)
    return ma
