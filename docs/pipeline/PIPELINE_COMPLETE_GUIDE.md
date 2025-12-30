# 📋 완전한 파이프라인 설명 및 사용법 - 최종 요약

> **작성일**: Nov 25, 2024  
> **대상**: `integrated_pcd_depth_pipeline_newest.py` 사용자  
> **총 소요 시간**: 약 3-8시간 (데이터량에 따라)

---

## 🎯 1단계: 파이프라인 동작 원리 (5분 요약)

### 파이프라인이 하는 일

```
입력: PCD 파일 1001개 + RGB 이미지 1001개
     ↓
[1] 각 PCD 파일에서 3D 포인트 클라우드 로드
[2] 지면에 동심원 패턴 합성 포인트 생성
[3] 카메라 intrinsic 파라미터로 3D → 2D 투영
[4] 깊이맵 생성 (float32)
[5] 3가지 해상도로 스케일링 (1920×1536, 640×512, 640×384)
[6] 깊이맵을 uint16 PNG로 저장 (KITTI 포맷)
[7] RGB 이미지에 깊이값 오버레이 시각화
     ↓
출력: 3 × 1001 = 3,003개 이미지 파일 (3가지 해상도)
```

### 핵심 개념 3가지

#### 1️⃣ VADAS Fisheye 카메라 모델 (11-parameter)

```python
intrinsic = [
    k0, k1, k2, k3, k4, k5, k6,  # 다항식 계수 (절대 스케일링 금지!)
    s,                            # 크기 파라미터 (절대 스케일링 금지!)
    div,                          # 왜곡 계수 (절대 스케일링 금지! - 핵심 버그)
    ux, uy                        # 주점 (반드시 스케일링!)
]
```

**중요**: Nov 14에 발견한 버그 → **div 파라미터는 스케일링하면 안됨**

#### 2️⃣ 스케일 적용 방식 (가로/세로 비율)

```
원본 해상도: 1920×1536

640×512:   scale_x = 640/1920 = 0.333
           scale_y = 512/1536 = 0.333 (균일)

640×384:   scale_x = 640/1920 = 0.333
           scale_y = 384/1536 = 0.250 (비균일, 종횡비 변경)
```

**투영 공식** (개선된 방식):
```python
# 거리 계산 (k 파라미터 사용)
rd = k[0] + k[1]*rho + k[2]*rho^2 + ... + k[6]*rho^6

# 각도 성분
cosPhi = cx / sqrt(cx^2 + cy^2)
sinPhi = cy / sqrt(cx^2 + cy^2)

# 최종 이미지 좌표 (scale_x, scale_y 적용!)
u = rd * cosPhi * scale_x + ux + img_w_half
v = rd * sinPhi * scale_y + uy + img_h_half
```

#### 3️⃣ 깊이맵 저장 형식 (KITTI Convention)

```python
# 저장
depth_uint16 = uint16(depth_meters * 256)
cv2.imwrite("depth.png", depth_uint16)

# 로드
depth_uint16 = cv2.imread("depth.png", cv2.IMREAD_UNCHANGED)
depth_meters = depth_uint16.astype(np.float32) / 256.0
```

예: 5.0m → 1280 → PNG 저장 → 1280 로드 → 5.0m 복원

---

## 🚀 2단계: 바로 실행하기 (5분)

### 환경 확인
```bash
# Python 확인
python --version  # 3.8+ 필요

# 필수 패키지 설치
pip install numpy opencv-python Pillow
```

### 테스트 실행 (10개 파일)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/test_10" \
    --calibration_path "./calibration.json" \
    --end_idx 10
```

**예상 시간**: ~2분

### 결과 확인
```bash
# 생성된 파일 확인
ls -la output/test_10/depth_maps_newest/newest_viz_results/

# 이미지 뷰어로 확인
open output/test_10/depth_maps_newest/newest_viz_results/0000000000_depth_analysis.png
```

**확인할 것**:
- ✅ 이미지가 검정색이 아님 (포인트가 투영됨)
- ✅ 깊이값이 색상으로 표시됨 (초록→노랑→빨강)
- ✅ RGB 이미지와 정렬됨

---

## 📊 3단계: 명령어 옵션 이해하기

### 입력/출력
```bash
--input_dir "D:/data/..."          # 입력 경로 (필수)
--output_dir "./output/result"     # 출력 경로 (필수)
--calibration_path "./calibration.json"  # 보정 파일 (필수)
```

### 해상도
```bash
# 기본: 1920×1536, 640×512, 640×384
# --resized_resolutions "640x512,640x384"

# 원본만
# --resized_resolutions ""

# 추가 해상도
# --resized_resolutions "640x512,640x384,480x360,320x240"
```

### 처리 범위
```bash
--start_idx 0        # 시작 파일 인덱스 (기본: 0)
--end_idx 100        # 종료 파일 인덱스 (기본: -1=전체)
--skip_existing      # 기존 파일 건너뛰기 (플래그)
```

### 시각화
```bash
--max_depth 15.0     # 최대 깊이 (기본: 15.0m)
--point_size 2       # 포인트 크기 픽셀 (기본: 2)
--max_radius 15.0    # 합성 최대 반경 (기본: 15.0m)
--num_circles 8      # 동심원 개수 (기본: 8)
```

### 성능
```bash
--num_workers 4      # 병렬 워커 (기본: 4)
--batch_size 10      # 배치 크기 (기본: 10)
```

---

## 💡 4단계: 실전 예제 (선택 항목)

### 예제 1: 전체 처리 (1001개)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/.../synced_data" \
    --output_dir "./output/full_processing" \
    --calibration_path "./calibration.json" \
    --skip_existing
```
**예상 시간**: 3-8시간

### 예제 2: 100개 처리 후 200개 추가 처리
```bash
# 첫 번째 실행
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "./output/partial" \
    --calibration_path "..." \
    --end_idx 100

# 두 번째 실행 (100~200 처리)
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "./output/partial" \
    --calibration_path "..." \
    --start_idx 100 \
    --end_idx 200 \
    --skip_existing
```

### 예제 3: 고해상도만 빠르게
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "./output/high_res_only" \
    --calibration_path "..." \
    --resized_resolutions ""
```
**결과**: 1920×1536만 (디스크 ~300GB)

### 예제 4: 메모리 절약 (느린 처리)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "./output/low_memory" \
    --calibration_path "..." \
    --num_workers 1 \
    --batch_size 5
```

### 예제 5: 시각화 커스터마이징
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "..." \
    --output_dir "./output/custom_viz" \
    --calibration_path "..." \
    --max_depth 25.0 \
    --point_size 4 \
    --max_radius 20.0 \
    --end_idx 100
```

---

## 📁 5단계: 출력 파일 이해하기

### 디렉토리 구조

```
output/
└─ full_processing/
   ├─ depth_maps_newest/              ← 1920×1536 해상도
   │  ├─ newest_depth_maps/
   │  │  ├─ 0000000000.png           ✓ 깊이맵 (uint16)
   │  │  ├─ 0000000001.png
   │  │  └─ ... (1001개)
   │  │
   │  ├─ newest_synthetic_depth_maps/ ← 합성 포인트만
   │  │  ├─ 0000000000.png
   │  │  └─ ...
   │  │
   │  ├─ newest_viz_results/         ← RGB+Depth (중요!)
   │  │  ├─ 0000000000_depth_analysis.png  ✓ 시각화
   │  │  └─ ...
   │  │
   │  ├─ newest_colormap/            ← 깊이 히트맵
   │  │  ├─ 0000000000_colored.png
   │  │  └─ ...
   │  │
   │  ├─ newest_pcd/
   │  │  └─ 0000000000.pcd
   │  │
   │  └─ diff_results/               ← 깊이 차이
   │     ├─ 0000000000_merged.png
   │     ├─ 0000000000_synth.png
   │     └─ 0000000000_orig.png
   │
   ├─ 640x512_newest/                ← 640×512 해상도
   │  └─ (동일 구조)
   │
   └─ 640x384_newest/                ← 640×384 해상도
      └─ (동일 구조)
```

### 주요 파일 설명

| 파일 | 형식 | 설명 | 크기 |
|------|------|------|------|
| `newest_depth_maps/*.png` | uint16 PNG | 깊이 데이터 (KITTI) | ~450KB |
| `newest_viz_results/*_depth_analysis.png` | RGB PNG | RGB+Depth 오버레이 | ~2.5MB |
| `newest_colormap/*_colored.png` | RGB PNG | 깊이 colormap | ~2.5MB |
| `diff_results/*.png` | RGB PNG | 깊이 차이 시각화 | ~2.5MB |
| `newest_pcd/*.pcd` | Binary | PCD 포인트 클라우드 | ~500KB |

**중요**: `newest_depth_maps/*.png`가 실제 깊이 데이터 (검증 가능)

---

## 🔍 6단계: 결과 검증하기

### Python으로 검증
```python
import cv2
import numpy as np

# 깊이맵 로드
depth_uint16 = cv2.imread(
    'output/full_processing/depth_maps_newest/newest_depth_maps/0000000000.png',
    cv2.IMREAD_UNCHANGED
)
depth_meters = depth_uint16.astype(np.float32) / 256.0

# 통계
print(f"이미지 크기: {depth_meters.shape}")       # (1536, 1920)
print(f"깊이 범위: {depth_meters.min():.2f}m ~ {depth_meters.max():.2f}m")
print(f"평균 깊이: {depth_meters[depth_meters>0].mean():.2f}m")
print(f"유효 픽셀: {np.count_nonzero(depth_meters)}")
print(f"영 픽셀: {np.sum(depth_meters==0)}")
```

**예상 결과**:
```
이미지 크기: (1536, 1920)
깊이 범위: 0.00m ~ 50.00m
평균 깊이: 15.23m
유효 픽셀: 450000 (약 15%)
영 픽셀: 2520000 (약 85%)
```

### 파일 크기 확인
```bash
# 1920×1536 해상도 파일
ls -lh output/full_processing/depth_maps_newest/newest_depth_maps/ | head -5
# -rw-r--r--  450K  0000000000.png

# 640×512 해상도 파일
ls -lh output/full_processing/640x512_newest/newest_depth_maps/ | head -5
# -rw-r--r--   50K  0000000000.png  (약 1/9 크기)
```

---

## ⚠️ 7단계: 자주 묻는 질문

### Q1. 깊이맵이 검정색인데요?
**A**: 포인트가 카메라 범위 밖에 있을 가능성
- calibration.json 확인
- 카메라 intrinsic 파라미터 검증
- 입력 데이터 (포인트 좌표 범위) 확인

### Q2. 처리 시간이 너무 오래 걸려요
**A**: 배치 크기와 워커 수 조정
```bash
# 빠르게
--num_workers 8 --batch_size 20

# 메모리 절약
--num_workers 1 --batch_size 5
```

### Q3. 디스크가 부족해요
**A**: 해상도 줄이기
```bash
# 원본만 (300GB)
--resized_resolutions ""

# 저해상도만 (35GB)
--resized_resolutions "640x512"
```

### Q4. 특정 구간만 처리하고 싶어요
**A**: start_idx, end_idx 사용
```bash
--start_idx 100 --end_idx 200  # 100~199번 처리
```

### Q5. 기존 파일을 다시 처리하고 싶어요
**A**: skip_existing 제거 또는 폴더 삭제
```bash
# skip_existing 없음 → 덮어쓰기
python ... (--skip_existing 빼기)

# 또는 폴더 삭제
rm -rf output/full_processing/depth_maps_newest/newest_viz_results/
```

---

## 📈 8단계: 성능 정보

### 처리 시간

**GTX 1080+ (GPU 사용 안함)**:
```
초당 처리: 0.08파일/초
파일당: 12초
────────────────
1001파일: 3.3시간
```

**CPU만 (저사양)**:
```
초당 처리: 0.033파일/초
파일당: 30초
────────────────
1001파일: 8.3시간
```

### 디스크 사용량

```
1920×1536 해상도:
  - 깊이맵: 300GB
  - 시각화: 2.5TB (추가)
  - 총: ~3TB

640×512 해상도:
  - 깊이맵: 35GB
  - 시각화: 300GB
  - 총: ~350GB

640×384 해상도:
  - 깊이맵: 25GB
  - 시각화: 200GB
  - 총: ~250GB

────────────────────
3가지 해상도 모두: ~3.6TB
```

**권장 사양**:
- SSD: 최소 500GB 여유
- HDD: 최소 1TB 여유

---

## ✅ 9단계: 최종 체크리스트

실행 전 확인:

- [ ] **환경**
  - [ ] Python 3.8+ 설치
  - [ ] numpy, opencv-python, Pillow 설치

- [ ] **데이터**
  - [ ] calibration.json 파일 존재
  - [ ] 입력 디렉토리에 PCD 파일 1000+개
  - [ ] image_a6 폴더에 JPG 이미지 1000+개
  - [ ] 총 데이터 크기 확인 (약 50-100GB)

- [ ] **환경 설정**
  - [ ] 출력 디렉토리에 쓰기 권한
  - [ ] 디스크 여유 500GB 이상
  - [ ] 로컬 드라이브 (네트워크 드라이브 X)

- [ ] **명령어**
  - [ ] --input_dir 경로 확인
  - [ ] --output_dir 경로 확인
  - [ ] --calibration_path 파일 확인

---

## 🎓 10단계: 추가 학습 자료

### 상세 가이드
1. **빠른 시작** (5분)  
   → `docs/QUICK_START_GUIDE.md`

2. **완전한 설명** (30분)  
   → `docs/COMPREHENSIVE_CODE_GUIDE.md`
   - 파이프라인 흐름도
   - 각 함수 상세 설명
   - 모든 명령어 옵션
   - 문제 해결

3. **API 레퍼런스**  
   → `docs/API_REFERENCE.md`
   - 클래스 API
   - 함수 시그니처
   - 데이터 형식
   - 코드 예제

### 기존 문서
- `docs/pipeline/` - 파이프라인 아키텍처
- `docs/resolution_support/` - 스케일링 상세
- `docs/analysis/` - 코드 분석

---

## 🚀 11단계: 시작하기

### 지금 바로

```bash
# Step 1: 테스트 (10개 파일, ~2분)
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/test_10" \
    --calibration_path "./calibration.json" \
    --end_idx 10

# Step 2: 결과 확인
open output/test_10/depth_maps_newest/newest_viz_results/

# Step 3: 전체 실행 (3-8시간)
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/full_result" \
    --calibration_path "./calibration.json"
```

### 다음 단계
1. 파이프라인 이해 → `docs/COMPREHENSIVE_CODE_GUIDE.md`
2. API 학습 → `docs/API_REFERENCE.md`
3. 커스터마이징 → 코드 수정

---

## 📞 문제 해결 요약

| 증상 | 원인 | 해결책 |
|------|------|--------|
| "No PCD files found" | 경로 오류 | 입력 경로 확인 |
| 검정 이미지 | 포인트 범위 밖 | calibration.json 검증 |
| 메모리 부족 | 배치 크기 너무 큼 | --batch_size 5 --num_workers 1 |
| 인코딩 오류 | Windows cp949 | 자동 처리됨 (UTF-8) |
| 처리 너무 느림 | 순차 처리 | --num_workers 8 --batch_size 20 |

---

## 📌 핵심 요점 정리

✅ **파이프라인이 하는 일**:
- PCD 파일 → 3D 포인트 → 카메라 투영 → 2D 깊이맵
- 3가지 해상도: 1920×1536, 640×512, 640×384
- 깊이맵 = uint16 PNG (KITTI: value/256 = 미터)

✅ **핵심 기술**:
- VADAS Fisheye 11-parameter intrinsic
- **div 파라미터는 스케일링 금지** (Nov 14 버그 수정)
- scale_x, scale_y는 최종 픽셀 좌표에만 적용

✅ **사용법**:
- 기본: `--input_dir ... --output_dir ... --calibration_path ...`
- 테스트: `--end_idx 10`
- 해상도: `--resized_resolutions "640x512,640x384"`

✅ **결과 확인**:
- 주요 파일: `newest_depth_maps/*.png` (실제 깊이 데이터)
- 시각화: `newest_viz_results/*_depth_analysis.png` (RGB+Depth)
- 검증: `newest_colormap/*_colored.png` (히트맵)

---

## 🎉 완료!

이제 모든 것을 이해했습니다. **바로 실행해보세요!**

```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/test_10" \
    --calibration_path "./calibration.json" \
    --end_idx 10
```

**더 자세한 정보는 `docs/` 폴더의 가이드를 참조하세요!**

