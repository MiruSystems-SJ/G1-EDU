"""예제 공용: 저장소 루트를 임포트 경로에 추가(설치 없이 실행 가능하게)."""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
