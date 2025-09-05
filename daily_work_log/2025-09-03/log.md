
## 2025년 9월 3일 수요일 작업 로그

### 1. LiDAR Closest Line Points 데이터 분석 및 피팅

**목표:** LiDAR에서 추출된 도로에 가장 가까운 점들(closest_line_points)을 활용하여, LiDAR 설치 위치 때문에 비어있는 가까운 도로 영역의 가상 PCD를 생성하기 위한 라인 피팅 모델을 도출.

**진행 내용:**

1.  **데이터 탐색:**
    *   `ncdb-cls-sample\synced_data\closest_line_points` 경로의 `.json` 파일 목록 확인. `.gitignore`에 의해 무시된 파일들을 포함하여 101개의 `.json` 파일 확인.
    *   샘플 파일(`0000000020.json`) 내용을 읽어 데이터 구조(x, y, z 좌표 리스트) 파악.

2.  **데이터 분석 스크립트 (`analyze_closest_line_points.py`) 작성 및 초기 분석:**
    *   모든 `.json` 파일을 읽어 x, y, z 데이터를 통합하고, 초기 통계 분석(평균, 최소, 최대, 표준 편차)을 수행하는 Python 스크립트 `analyze_closest_line_points.py`를 새로 작성.
    *   스크립트 실행 결과, 총 8656개의 포인트 로드 및 초기 통계 확인.

3.  **이상치 제거 및 라인 피팅 시도 (Open3D 사용 시도):**
    *   `analyze_closest_line_points.py` 스크립트에 Open3D를 이용한 통계적 이상치 제거 및 2차 다항식 피팅 로직 추가.
    *   **문제 발생:** `ImportError: DLL load failed while importing pybind: 지정된 모듈을 찾을 수 없습니다.` 오류 발생. Open3D 라이브러리 로딩 문제로 추정.
    *   **해결 시도:** `pip uninstall open3d` 후 `pip install open3d` 재설치 시도. `OSError` 발생으로 실패.
    *   **사용자 조치:** 사용자께서 Open3D 관련 잔여 파일 수동 삭제 후 재설치 요청.
    *   **해결 시도:** `pip cache purge` 후 `pip install open3d` 재시도. `Requirement already satisfied` 메시지에도 불구하고 `ImportError` 지속.
    *   **해결 시도:** `point_cloud_env` 콘다 환경에서 실행 시도. 동일한 `ImportError` 발생.
    *   **문제 발생:** `open3d` 임포트 라인 제거 후 스크립트 실행 시 `AttributeError: 'open3d.cpu.pybind.geometry.PointCloud' object has no attribute 'remove_statistical_outliers'` 오류 발생. `open3d` 참조 문제 지속.

4.  **이상치 제거 및 라인 피팅 (NumPy/Scikit-learn으로 전환):**
    *   `analyze_closest_line_points.py` 스크립트에서 `open3d` 임포트 라인 완전히 제거.
    *   NumPy를 이용한 Z-score 기반 이상치 제거 로직 유지.
    *   `scikit-learn`의 `RANSACRegressor`와 `PolynomialFeatures(degree=2)`를 사용하여 2차 다항식 라인 피팅 로직 추가.
    *   **문제 발생:** `scikit-learn` 설치 시 `numpy` 관련 `OSError` 발생. 하지만 `scikit-learn`은 `Requirement already satisfied`로 표시되어 일단 진행.
    *   **최종 결과:** 스크립트 성공적으로 실행.
        *   Z-score 이상치 제거 후 8553개의 인라이어 포인트 확인.
        *   **RANSAC 2차 다항식 피팅 결과:** `y = 1.2621x^2 + 0.0000x + 3.5260` (계수: `[1.2621, 0.0, 3.5260]`)
        *   RANSAC 인라이어 포인트 수: 1255개.
        *   RANSAC 인라이어의 평균 Z-좌표: -0.8634.

**다음 단계:** 피팅된 2차 다항식 모델과 평균 Z-좌표를 기반으로 LiDAR에 더 가까운 가상의 PCD를 생성하는 로직 구현 예정.
