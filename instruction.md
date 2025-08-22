Depth Anything + VADAS Pipeline (Draft)
목표 기능

Distorted → Undistorted 변환 (VADAS Model 사용)

입력: Fisheye 왜곡 영상

처리: VADAS 모델을 이용해 Distorted Image Plane → Normalized Plane → Undistorted Image Plane 변환 수행

Normalized Plane 중간 단계 처리

Undistortion 과정에서 Normalized Plane 상의 중간 데이터를 확보

이후 Pixel Plane으로 다시 매핑 가능

Depth Anything v2 수행

Undistorted 영상 입력 → Depth Anything v2 모델로 Depth 추정 수행

Undistorted Depth → Distorted Depth 변환

추정된 Depth 결과를 다시 Undistorted Plane → Normalized Plane → Distorted Image Plane으로 매핑 (VADAS Model 기반)

Debugging 및 Crop 기능

각 단계별 결과를 시각화(윈도우 창으로 출력)

특히 Undistorted Normalized Plane 상에서 관심 영역을 지정하여 Crop 가능하도록 구현 (LDC 방식 참조)

예상 구현 전략

단계별 모듈화

distort_to_undistort() : Distorted → Undistorted 변환

undistort_to_distort() : Undistorted → Distorted 변환

run_depth_anything_v2() : Depth Anything v2 실행

debug_visualization() : 각 단계 디버깅을 위한 시각화

crop_in_normalized_plane() : Normalized Plane 기반 Crop

중간 데이터 저장/출력

Normalized 좌표계 결과를 numpy 형태로 저장

Pixel Plane 변환 후 시각화

디버깅 편의성

OpenCV imshow + 키 입력으로 단계 전환

각 단계별 저장 옵션 (--save_intermediate)

핵심 포인트

VADAS Model 기반 Backward mapping 수행

Depth Anything v2는 Undistorted Image Plane(Pixel Plane)에서만 수행

LDC된 이미지를 원하는 크기만큼 Crop 할 수 있어야함

전체 파이프라인이 시각적 검증 가능해야 함