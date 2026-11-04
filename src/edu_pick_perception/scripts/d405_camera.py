#!/usr/bin/env python3
"""RealSense D405 카메라 노드 (ROS 1) — 깊이+컬러를 격자형 점군으로 발행한다.

■ 왜 직접 만들었나
  ROS 1 용 공식 드라이버(realsense2_camera 2.3.2)는 D405 를 지원하지 않는다
  ("Unsupported device! Product ID: 0x0B5B" — ROS 1 판 개발이 그 전에 멈췄다).
  카메라 자체는 RealSense 파이썬 라이브러리(pyrealsense2)로 바로 읽을 수 있어, 이 실습에 필요한 것
  (컬러 정렬된 격자형 점군)만 만든다. ROS 2 판의 realsense2_camera 와 같은 토픽·형식을 낸다.

■ 발행
  /camera/d405/depth/color/points   (sensor_msgs/PointCloud2, 격자형 H×W, x·y·z·rgb)
  TF  d405_link → d405_color_optical_frame   (고정 · 광학 프레임 = z 앞, x 오른쪽, y 아래)

■ 파라미터
  ~serial   카메라가 여러 대일 때 시리얼 (빈 값 = 아무거나)
  ~width ~height ~fps   스트림 (기본 848×480 @ 15)
  ~rate     점군 발행 횟수 [Hz] (기본 5 — 인식 노드는 초당 3번 처리)
"""
import numpy as np
import pyrealsense2 as rs
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster

TOPIC = "/camera/d405/depth/color/points"
LINK, OPTICAL = "d405_link", "d405_color_optical_frame"


def main():
    rospy.init_node("d405_camera")
    serial = str(rospy.get_param("~serial", ""))
    w, h, fps = int(rospy.get_param("~width", 848)), int(rospy.get_param("~height", 480)), int(rospy.get_param("~fps", 15))
    rate = float(rospy.get_param("~rate", 5.0))

    cfg = rs.config()
    if serial:
        cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    cfg.enable_stream(rs.stream.color, w, h, rs.format.rgb8, fps)
    pipe = rs.pipeline()
    try:
        prof = pipe.start(cfg)
    except RuntimeError as e:
        rospy.logfatal(f"D405 를 열 수 없습니다 ({e}) — USB 연결·다른 프로그램이 카메라를 쓰는지 확인")
        raise SystemExit(1)
    dev = prof.get_device()
    scale = dev.first_depth_sensor().get_depth_scale()            # 깊이 단위 [m] (D405 는 0.0001)
    align = rs.align(rs.stream.color)                              # 깊이를 컬러 영상 좌표에 맞춘다
    intr = prof.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    rospy.loginfo(f"D405 시작 — {dev.get_info(rs.camera_info.name)} 시리얼 {dev.get_info(rs.camera_info.serial_number)} "
                  f"{w}×{h}@{fps} · 깊이 단위 {scale * 1000:.2f} mm")

    # 고정 TF: 카메라 몸체(d405_link, x 앞) → 컬러 광학 프레임(z 앞).  D405 의 컬러는 깊이와 같은 영상기라 위치 차 0
    ext = prof.get_stream(rs.stream.depth).get_extrinsics_to(prof.get_stream(rs.stream.color))
    st = TransformStamped()
    st.header.stamp = rospy.Time.now()
    st.header.frame_id, st.child_frame_id = LINK, OPTICAL
    # 광학 → 몸체 좌표 (x 앞 = 광학 z, y 왼쪽 = −광학 x, z 위 = −광학 y)
    t = ext.translation
    st.transform.translation.x, st.transform.translation.y, st.transform.translation.z = t[2], -t[0], -t[1]
    st.transform.rotation.x, st.transform.rotation.y = -0.5, 0.5
    st.transform.rotation.z, st.transform.rotation.w = -0.5, 0.5
    StaticTransformBroadcaster().sendTransform(st)

    # 픽셀마다 방향 (x/z, y/z) — 한 번만 계산
    uu, vv = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
    rx, ry = (uu - intr.ppx) / intr.fx, (vv - intr.ppy) / intr.fy
    pub = rospy.Publisher(TOPIC, PointCloud2, queue_size=2)
    fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32 if n != "rgb" else PointField.UINT32, count=1)
              for i, n in enumerate(["x", "y", "z", "rgb"])]
    buf = np.empty((h, w), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])
    period, last = 1.0 / rate, 0.0
    try:
        while not rospy.is_shutdown():
            frames = pipe.wait_for_frames(5000)
            now = rospy.Time.now()
            if (now.to_sec() - last) < period or pub.get_num_connections() == 0:
                continue
            last = now.to_sec()
            frames = align.process(frames)
            d, c = frames.get_depth_frame(), frames.get_color_frame()
            if not d or not c:
                continue
            z = np.asanyarray(d.get_data()).astype(np.float32) * scale
            z[z <= 0] = np.nan                                     # 0 = 깊이 없음
            col = np.asanyarray(c.get_data())
            buf["x"], buf["y"], buf["z"] = rx * z, ry * z, z
            buf["rgb"] = (col[..., 0].astype(np.uint32) << 16) | (col[..., 1].astype(np.uint32) << 8) | col[..., 2]
            msg = PointCloud2(height=h, width=w, fields=fields, is_bigendian=False, point_step=16, row_step=16 * w,
                              is_dense=False, data=buf.tobytes())
            msg.header.stamp, msg.header.frame_id = now, OPTICAL
            pub.publish(msg)
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
