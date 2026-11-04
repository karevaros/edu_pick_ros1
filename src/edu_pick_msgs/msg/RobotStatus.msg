# 제어박스 상태 (rb_driver 가 데이터 채널 5001 에서 읽어 발행)
Header header
bool connected               # 제어박스 데이터 채널 연결
int32 robot_state            # 1 = 정지(idle) · 2 = 일시정지/사고로 멈춤 · 3 = 움직이는 중
int32 task_state
bool simulation              # true = 제어박스 Simulation 모드(팔 안 움직임)
float32 speed_bar            # 제어박스 속도바 [0~1]
bool collision               # 충돌 감지로 멈춤
bool self_collision
bool soft_estop
bool emergency_stop
bool inbox_trap              # 인박스(안전 영역) 위반
