# integrated_pcd_depth_pipeline_newest.py - 빠른 시작 가이드

## 🚀 5분 안에 실행하기

### Step 1: 환경 확인
```bash
# Python 버전 확인
python --version  # 3.8 이상 필요

# 필수 패키지 설치
pip install numpy opencv-python Pillow

# 패키지 설치 확인
python -c "import cv2, numpy, PIL; print('OK')"
```

### Step 2: 데이터 준비
```bash
# 디렉토리 구조
D:/data/
├─ ncdb-cls/
│  └─ 2025-07-11_15-00-27_410410_A/
│     └─ synced_data/
│        ├─ image_a6/              # RGB 이미지 (000000.jpg 등)
│        ├─ pcd/                   # PCD 파일 (0000000000.pcd 등)
│        └─ calibration.json        # 보정 정보

# 파일 개수 확인
ls D:/data/ncdb-cls/.../synced_data/pcd/*.pcd | wc -l  # 1001개
ls D:/data/ncdb-cls/.../synced_data/image_a6/*.jpg | wc -l
```

### Step 3: 빠른 테스트 (10개 파일만)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/quick_test" \
    --calibration_path "./calibration.json" \
    --end_idx 10
```

### Step 4: 결과 확인
```bash
# 생성된 파일 확인
ls -la output/quick_test/depth_maps_newest/newest_depth_maps/ | head -5
ls -la output/quick_test/depth_maps_newest/newest_viz_results/ | head -5

# 이미지 뷰어로 확인
# output/quick_test/depth_maps_newest/newest_viz_results/0000000000_depth_analysis.png
```

---

## 📋 명령어 치트시트

### 기본 실행
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir <입력경로> \
    --output_dir <출력경로> \
    --calibration_path <보정파일>
```

### 부분 처리
```bash
# 처음 100개만
--end_idx 100

# 100~200번째
--start_idx 100 --end_idx 200

# 기존 파일 건너뛰기
--skip_existing
```

### 해상도 설정
```bash
# 기본 (3가지): 1920x1536, 640x512, 640x384
# --resized_resolutions "640x512,640x384"

# 원본만 (크기 조정 없음)
# --resized_resolutions ""

# 커스텀 해상도
# --resized_resolutions "640x512,480x360,320x240"
```

### 합성 포인트
```bash
# 기본: 반경 15m, 8개 원, 각 100포인트
# --max_radius 15.0 --num_circles 8 --points_per_circle 100

# 더 많이 생성
# --max_radius 20.0 --num_circles 10 --points_per_circle 50
```

### 시각화
```bash
# 기본: 15m까지 표시
# --max_depth 15.0

# 더 먼 거리까지
# --max_depth 25.0

# 포인트 크기
# --point_size 2  (작음)
# --point_size 4  (중간)
# --point_size 8  (큼)
```

### 성능
```bash
# 빠른 처리 (병렬)
# --num_workers 8 --batch_size 20

# 느린 처리 (메모리 절약)
# --num_workers 1 --batch_size 5
```

---

## 🎯 실전 예제

### 예제 1: 전체 데이터 처리
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/full_processing" \
    --calibration_path "./calibration.json" \
    --skip_existing
```
**예상 시간**: ~2-3시간 (1001개 파일)

### 예제 2: 테스트 실행
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/test_run" \
    --calibration_path "./calibration.json" \
    --end_idx 50
```
**예상 시간**: ~8분 (50개 파일)

### 예제 3: 고해상도만
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/high_res" \
    --calibration_path "./calibration.json" \
    --resized_resolutions ""
```
**출력**: 1920×1536만 (약 300GB 디스크)

### 예제 4: 저해상도만
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/low_res" \
    --calibration_path "./calibration.json" \
    --resized_resolutions "640x512"
```
**출력**: 640×512만 (약 100GB 디스크)

### 예제 5: 추가 해상도
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/multi_res" \
    --calibration_path "./calibration.json" \
    --resized_resolutions "640x512,640x384,480x360,320x240"
```
**출력**: 5가지 해상도 (약 600GB 디스크)

### 예제 6: 커스텀 시각화
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/custom_viz" \
    --calibration_path "./calibration.json" \
    --point_size 4 \
    --max_depth 20.0 \
    --end_idx 100
```
**결과**: 더 큰 포인트, 더 먼 거리까지 시각화

### 예제 7: 재개 (이전 실행 계속)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/full_processing" \
    --calibration_path "./calibration.json" \
    --start_idx 100 \
    --skip_existing
```
**설명**: 100번째부터 재개, 기존 파일은 건너뜀

---

## 📊 출력 파일 설명

### 생성되는 폴더 구조

```
output/
└─ quick_test/
   ├─ depth_maps_newest/               ← 1920×1536 해상도
   │  ├─ newest_depth_maps/            ← 실제 깊이 데이터 (PNG)
   │  │  ├─ 0000000000.png            ✓ 깊이맵 (uint16)
   │  │  ├─ 0000000001.png
   │  │  └─ ...
   │  │
   │  ├─ newest_synthetic_depth_maps/  ← 합성 포인트만
   │  │  ├─ 0000000000.png
   │  │  └─ ...
   │  │
   │  ├─ newest_viz_results/           ← RGB + Depth 오버레이 (중요!)
   │  │  ├─ 0000000000_depth_analysis.png  ✓ 검증용
   │  │  └─ ...
   │  │
   │  ├─ newest_colormap/              ← 깊이 히트맵
   │  │  ├─ 0000000000_colored.png
   │  │  └─ ...
   │  │
   │  ├─ newest_pcd/                   ← PCD 파일들
   │  │  ├─ 0000000000.pcd
   │  │  └─ ...
   │  │
   │  └─ diff_results/                 ← 깊이 차이 시각화
   │     ├─ 0000000000_merged.png
   │     ├─ 0000000000_synth.png
   │     ├─ 0000000000_orig.png
   │     └─ ...
   │
   ├─ 640x512_newest/                  ← 640×512 해상도 (균일)
   │  └─ (동일 구조)
   │
   └─ 640x384_newest/                  ← 640×384 해상도 (비균일)
      └─ (동일 구조)
```

### 주요 파일 설명

| 파일명 | 형식 | 설명 |
|--------|------|------|
| `0000000000.png` | uint16 PNG | 깊이맵 (KITTI: 값/256=미터) |
| `0000000000_colored.png` | RGB PNG | 깊이 colormap (Jet) |
| `0000000000_depth_analysis.png` | RGB PNG | RGB + Depth 산점도 |
| `0000000000_merged.png` | RGB PNG | 병합 깊이 차이 |
| `0000000000_synth.png` | RGB PNG | 합성 깊이 차이 |
| `0000000000_orig.png` | RGB PNG | 원본 깊이 차이 |
| `0000000000.pcd` | Binary | PCD 포인트 클라우드 |

---

## 🔍 결과 확인 방법

### 1. 디스크 사용량 확인
```bash
# 전체 크기
du -sh output/quick_test/

# 해상도별 크기
du -sh output/quick_test/depth_maps_newest/
du -sh output/quick_test/640x512_newest/
du -sh output/quick_test/640x384_newest/

# 파일 개수
find output/quick_test -name "*.png" | wc -l
```

### 2. 파일 크기 검증
```bash
# 깊이맵 (1920×1536) - 약 450KB
ls -lh output/quick_test/depth_maps_newest/newest_depth_maps/0000000000.png

# RGB+Depth (1920×1536) - 약 2.5MB
ls -lh output/quick_test/depth_maps_newest/newest_viz_results/0000000000_depth_analysis.png

# 깊이맵 (640×512) - 약 50KB
ls -lh output/quick_test/640x512_newest/newest_depth_maps/0000000000.png

# RGB+Depth (640×512) - 약 300KB
ls -lh output/quick_test/640x512_newest/newest_viz_results/0000000000_depth_analysis.png
```

### 3. 깊이맵 검증 (Python)
```python
import cv2
import numpy as np

# 깊이맵 로드
depth_uint16 = cv2.imread('output/quick_test/depth_maps_newest/newest_depth_maps/0000000000.png', cv2.IMREAD_UNCHANGED)
depth_meters = depth_uint16.astype(np.float32) / 256.0

print(f"이미지 크기: {depth_meters.shape}")
print(f"최소 깊이: {depth_meters.min():.2f}m")
print(f"최대 깊이: {depth_meters.max():.2f}m")
print(f"평균 깊이: {depth_meters.mean():.2f}m")
print(f"유효 픽셀: {np.count_nonzero(depth_meters)}")
print(f"영 픽셀: {np.sum(depth_meters == 0)}")
```

### 4. 이미지 뷰어로 확인
```bash
# Windows
explorer output\quick_test\depth_maps_newest\newest_viz_results\

# Mac
open -a Preview output/quick_test/depth_maps_newest/newest_viz_results/

# Linux
eog output/quick_test/depth_maps_newest/newest_viz_results/ &
```

---

## ⚠️ 일반적인 문제와 해결책

### 문제: "cannot find calibration.json"
```bash
# 해결: 절대 경로 사용
--calibration_path "C:/Users/seok436/path/to/calibration.json"

# 또는 현재 디렉토리 확인
ls -la | grep calibration.json
```

### 문제: "No PCD files found"
```bash
# PCD 파일 위치 확인
ls D:/data/ncdb-cls/.../synced_data/pcd/*.pcd | head -5

# 입력 경로 수정
--input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data"
```

### 문제: "Depth map is all black"
```bash
# 원인: 포인트가 카메라 범위 밖
# 해결: calibration.json 확인, 카메라 좌표계 점검
```

### 문제: "Out of memory"
```bash
# 해결: 배치 크기 축소
--batch_size 5 --num_workers 1
```

### 문제: "Encoding error (cp949)"
```bash
# 자동 처리됨 (Windows)
# 수동으로 하려면:
python -c "import sys; sys.stdout.reconfigure(encoding='utf-8')"
```

---

## 📈 성능 참고값

### 처리 시간 (1개 파일당)
```
고사양 컴퓨터 (GTX 1080+):
  - 1920×1536 해상도: ~8초
  - 640×512 해상도: ~2초
  - 640×384 해상도: ~1.5초
  - 합성 포인트: ~0.5초
  ────────────────────
  총: ~12초/파일 × 1001파일 ≈ 3.3시간

저사양 컴퓨터 (CPU만):
  - 1920×1536 해상도: ~20초
  - 640×512 해상도: ~5초
  - 640×384 해상도: ~3.5초
  - 합성 포인트: ~2초
  ────────────────────
  총: ~30초/파일 × 1001파일 ≈ 8.3시간
```

### 디스크 사용량
```
해상도별 디스크 사용량:
  1920×1536: ~300GB (모든 파일 포함)
  640×512:   ~35GB  (모든 파일)
  640×384:   ~25GB  (모든 파일)
  ────────────────
  총 3가지: ~360GB

추천 설정:
  - SSD: 최소 500GB 여유 필요
  - HDD: 최소 1TB 여유 권장
```

---

## ✅ 체크리스트

실행 전에 확인하세요:

```
[ ] Python 3.8 이상 설치
[ ] numpy, opencv-python, Pillow 설치됨
[ ] calibration.json 파일 있음
[ ] 입력 디렉토리 경로 확인됨
[ ] PCD 파일 1000+ 개 있음
[ ] image_a6 폴더에 RGB 이미지 있음
[ ] 출력 디렉토리 쓰기 권한 있음
[ ] 디스크 여유 공간 충분함 (500GB+)
[ ] 네트워크 드라이브가 아님 (로컬 디스크)
```

---

## 🎓 다음 단계

### 1. 처음 실행
```bash
# 10개 파일로 테스트
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "./output/test" \
    --calibration_path "..." \
    --end_idx 10
```

### 2. 결과 검증
```bash
# 생성된 이미지 확인
# output/test/depth_maps_newest/newest_viz_results/
```

### 3. 전체 실행
```bash
# 확신하면 전체 1001개 처리
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "./output/full" \
    --calibration_path "..."
```

### 4. 분석
```python
# 생성된 깊이맵 분석
import cv2, numpy as np

depth = cv2.imread('output/full/depth_maps_newest/newest_depth_maps/0000000000.png', cv2.IMREAD_UNCHANGED)
depth = depth.astype(np.float32) / 256.0

# 통계
print(f"범위: {depth.min():.1f}m ~ {depth.max():.1f}m")
print(f"평균: {depth[depth>0].mean():.1f}m")
```

---

## 📞 추가 정보

자세한 설명은 `COMPREHENSIVE_CODE_GUIDE.md` 참조

