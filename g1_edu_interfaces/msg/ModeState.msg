# FSM 모드와 마지막 에러 (교재 5장 GetMode / GetLastError 대응)
uint32 tick
string mode    # damp | standing_up | balance_stand | walk
string error   # 예: FALL_DETECTED — ... (없으면 빈 문자열)
