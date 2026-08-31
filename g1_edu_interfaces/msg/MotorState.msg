# 관절 하나의 low state (실기체 LowState.motor_state 항목의 축약판)
float32 q            # 관절 각도 [rad]
float32 dq           # 관절 각속도 [rad/s]
float32 tau_est      # 추정 토크 [Nm]
float32 temperature  # 모터 온도 [C] (시뮬: 토크 기반 근사)
