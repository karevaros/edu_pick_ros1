# 인식된 물체 1개 (좌표계 = DetectedObjectArray.header.frame_id, 보통 world = 로봇 베이스)
string id            # obj_0, obj_1 ...
string shape         # "round"(캔·병·공 — 위에서 보면 원) / "box"(박스)
geometry_msgs/Point top_center   # 물체 윗면 중심 [m]
float64 yaw          # 그리퍼가 닫히는 방향[rad] = 물체의 짧은 변 방향. round 는 NaN(방향 무관)
float64 width        # 짧은 변(=그리퍼로 집을 폭) [m]
float64 length       # 긴 변 [m] (round 는 width 와 같음)
float64 height       # 테이블 위 높이 [m]
bool graspable       # 집을 수 있나 (폭이 그리퍼 벌림 이하 · 시야에 다 들어옴)
string note          # 집지 않는 이유 등 (예: "너무 넓음", "가장자리에 잘림")
