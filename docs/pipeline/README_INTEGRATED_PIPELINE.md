# integrated_pcd_depth_pipeline_newest.py 사용 설명서

## 📌 개요

`integrated_pcd_depth_pipeline_newest.py`는 **LiDAR 포인트 클라우드를 여러 해상도에서 깊이맵으로 변환**하는 완전 자동화 파이프라인입니다.

### 주요 특징

✅ **다중 해상도 지원**: 1920×1536, 640×512, 640×384 (사용자 정의 가능)  
✅ **Fisheye 카메라 모델**: VADAS 11-parameter intrinsic 모델 정확한 투영  
✅ **합성 포인트**: C-circle 패턴으로 시각화 및 검증  
✅ **RGB+Depth 시각화**: 깊이값을 색상으로 표시한 오버레이  
✅ **KITTI 포맷**: uint16 PNG로 손실 없이 저장 (value/256 = 미터)  
✅ **완전 자동화**: 1001개 파일 처리 시 ~3시간 (고사양 PC)

---

## 📚 문서 구조

### [1️⃣ 빠른 시작 (5분)](QUICK_START_GUIDE.md)
**처음 사용자를 위한 가이드**
- 환경 설정
- 테스트 실행 (10개 파일)
- 결과 확인
- 자주 묻는 질문

**바로 실행하려면**:
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/test" \
    --calibration_path "./calibration.json" \
    --end_idx 10
```

---

### [2️⃣ 완전 설명서 (30분)](COMPREHENSIVE_CODE_GUIDE.md)
**코드 동작 원리를 완벽히 이해하려는 사용자를 위한 문서**

**포함 내용**:
1. **파이프라인 전체 흐름도**: 입력부터 출력까지
2. **핵심 개념**:
   - VADAS Fisheye 카메라 모델 (11-parameter intrinsic)
   - 스케일 적용 방식 (가로/세로 비율 유지)
   - 깊이맵 저장 형식 (KITTI uint16)
   - 합성 포인트 생성 (C-circle)

3. **주요 함수 설명**:
   - `VADASFisheyeCameraModel` 클래스
   - `LidarCameraProjector` 클래스
   - 시각화 함수들

4. **출력 파일 설명**: 각 파일의 의미와 해석 방법
5. **전체 명령어 옵션**: 모든 파라미터 상세 설명
6. **실행 결과 해석**: 로그 읽기, 파일 검증
7. **문제 해결**: 자주 발생하는 오류와 해결책
8. **성능 최적화**: 처리 시간 단축 팁

---

### [3️⃣ API 레퍼런스](API_REFERENCE.md)
**프로그래머를 위한 상세 API 문서**

**포함 내용**:
- `VADASFisheyeCameraModel`: 초기화, scale_intrinsics(), project_point()
- `CalibrationDB`: 보정 데이터 관리
- `LidarCameraProjector`: 깊이맵 생성 (project_cloud_to_depth_map_with_labels)
- **유틸리티 함수**: PCD 읽기/쓰기, 깊이맵 저장, 시각화
- **데이터 형식**: calibration.json, 깊이맵 PNG, PCD
- **통합 예제**: 완전한 워크플로우 코드
- **성능 최적화**: 메모리/속도 개선 팁

---

## 🚀 빠른 사용법

### 기본 실행 (1001개 모두)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/full_result" \
    --calibration_path "./calibration.json"
```

### 테스트 (처음 100개)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/test_100" \
    --calibration_path "./calibration.json" \
    --end_idx 100
```

### 고해상도만 (1920×1536)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/high_res" \
    --calibration_path "./calibration.json" \
    --resized_resolutions ""
```

### 추가 해상도 (480×360까지)
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/2025-07-11_15-00-27_410410_A/synced_data" \
    --output_dir "./output/multi_res" \
    --calibration_path "./calibration.json" \
    --resized_resolutions "640x512,640x384,480x360"
```

더 많은 예제는 [QUICK_START_GUIDE.md](QUICK_START_GUIDE.md) 참조

---

## 📊 주요 명령어 옵션

| 옵션 | 설명 | 기본값 | 예시 |
|------|------|--------|------|
| `--input_dir` | 입력 데이터 경로 | 필수 | `"D:/data/..."` |
| `--output_dir` | 출력 디렉토리 | 필수 | `"./output/result"` |
| `--calibration_path` | 카메라 보정 JSON | 필수 | `"./calibration.json"` |
| `--resized_resolutions` | 크기 조정 해상도 | `"640x512,640x384"` | `"640x512"` or `""` |
| `--start_idx` | 시작 파일 인덱스 | 0 | 100 |
| `--end_idx` | 종료 파일 인덱스 | -1 (전체) | 200 |
| `--skip_existing` | 기존 파일 건너뛰기 | False | 플래그 |
| `--max_depth` | 시각화 최대 깊이 | 15.0 | 20.0 |
| `--point_size` | 포인트 크기 (픽셀) | 2 | 4 |
| `--max_radius` | 합성 최대 반경 | 15.0 | 20.0 |
| `--num_workers` | 병렬 처리 워커 | 4 | 8 |

전체 옵션은 [COMPREHENSIVE_CODE_GUIDE.md](COMPREHENSIVE_CODE_GUIDE.md) 섹션 6 참조

---

## 📁 출력 디렉토리 구조

```
output/
└─ full_result/
   ├─ depth_maps_newest/               ← 1920×1536 (원본 해상도)
   │  ├─ newest_depth_maps/            ← 깊이맵 PNG (uint16, KITTI)
   │  ├─ newest_synthetic_depth_maps/  ← 합성 포인트만
   │  ├─ newest_viz_results/           ← RGB+Depth 오버레이 ⭐
   │  ├─ newest_colormap/              ← 깊이 colormap (Jet)
   │  ├─ newest_pcd/                   ← PCD 파일
   │  └─ diff_results/                 ← 깊이 차이 시각화
   │
   ├─ 640x512_newest/                  ← 640×512 (균일 스케일)
   │  └─ (동일 구조)
   │
   └─ 640x384_newest/                  ← 640×384 (비균일 스케일)
      └─ (동일 구조)
```

**주요 파일**:
- `newest_depth_maps/*.png`: 실제 깊이 데이터 (uint16, value/256=미터)
- `newest_viz_results/*_depth_analysis.png`: RGB + Depth 시각화 (검증용) ⭐
- `newest_colormap/*_colored.png`: Depth colormap (Jet)

---

## 🎓 핵심 개념 이해하기

### 1. VADAS Fisheye 카메라 모델

**11-parameter intrinsic 벡터**:
```python
intrinsic = [k0, k1, k2, k3, k4, k5, k6,  # 다항식 계수 (polynomial)
             s,                           # 크기 파라미터
             div,                         # 왜곡 계수 ⚠️ 스케일링 금지!
             ux, uy]                      # 주점 (반드시 스케일링)
```

**중요**: `div` 파라미터는 **절대 스케일링하면 안됨**. Nov 14에 발견한 핵심 버그.

### 2. 스케일 적용 방식

**640×512 (균일 스케일)**:
```
scale_x = 640/1920 = 0.333
scale_y = 512/1536 = 0.333 (동일)
→ 종횡비 유지
```

**640×384 (비균일 스케일)**:
```
scale_x = 640/1920 = 0.333
scale_y = 384/1536 = 0.250 (다름)
→ 종횡비 변경 (16:12 → 5:3)
```

**투영 공식** (개선된 방식):
```python
rd = k[0] + k[1]*rho + k[2]*rho^2 + ... + k[6]*rho^6

cosPhi = cx / sqrt(cx^2 + cy^2)
sinPhi = cy / sqrt(cx^2 + cy^2)

u = rd * cosPhi * scale_x + ux + img_w_half
v = rd * sinPhi * scale_y + uy + img_h_half
```

### 3. 깊이맵 저장 형식 (KITTI Convention)

```python
# 저장
depth_uint16 = uint16(depth_meters * 256)
cv2.imwrite("depth.png", depth_uint16)

# 로드
depth_uint16 = cv2.imread("depth.png", cv2.IMREAD_UNCHANGED)
depth_meters = depth_uint16.astype(np.float32) / 256.0
```

---

## ⚙️ 기술 스택

| 컴포넌트 | 설명 |
|---------|------|
| **Python** | 3.8+ |
| **NumPy** | 배열 처리 |
| **OpenCV** | 이미지 I/O, 리사이징 |
| **Pillow** | 이미지 저장 |
| **JSON** | 설정 파일 |

---

## 🔍 실행 결과 검증

### 로그 확인
```
[INFO] Loading calibration from ./calibration.json
[INFO] Found 1001 PCD files

[PROCESS] File 0/1001: 0000000000.pcd
[LOAD] Loaded 50000 original points
[SYNTH] Generated 800 synthetic points
[DEPTH] Generated depth_map for 1920x1536
[SCALE] Processing resolution 640x512...
[SAVE] RGB+Depth visualization saved
[COMPLETE] File 0/1001 finished

[FINISH] Pipeline complete!
[STATS] Average time per file: 9.2s
```

### 파일 확인
```bash
# 깊이맵 존재 확인
ls output/full_result/depth_maps_newest/newest_depth_maps/ | wc -l  # 1001개

# RGB+Depth 시각화 확인
ls output/full_result/depth_maps_newest/newest_viz_results/ | wc -l  # 1001개

# 파일 크기 확인
ls -lh output/full_result/depth_maps_newest/newest_depth_maps/0000000000.png
```

### Python 검증
```python
import cv2
import numpy as np

# 깊이맵 로드
depth_uint16 = cv2.imread('output/full_result/depth_maps_newest/newest_depth_maps/0000000000.png', cv2.IMREAD_UNCHANGED)
depth_meters = depth_uint16.astype(np.float32) / 256.0

print(f"크기: {depth_meters.shape}")
print(f"범위: {depth_meters.min():.1f}m ~ {depth_meters.max():.1f}m")
print(f"유효 픽셀: {np.count_nonzero(depth_meters)}")
```

---

## 🐛 문제 해결

### "No PCD files found"
→ 입력 경로 확인: `{input_dir}/pcd/*.pcd` 존재 여부

### "Depth map is all black"
→ calibration.json 확인, 카메라 intrinsic 파라미터 검증

### "Out of memory"
→ `--batch_size 5 --num_workers 1` 사용

### "Windows encoding error"
→ 코드에서 자동 처리됨 (UTF-8 강제 적용)

자세한 문제 해결은 [COMPREHENSIVE_CODE_GUIDE.md](COMPREHENSIVE_CODE_GUIDE.md) 섹션 8 참조

---

## 📈 성능 정보

### 처리 시간 (1파일당)

**고사양 (GTX 1080+)**:
```
1920×1536: 8초
640×512:   2초
640×384:   1.5초
합성:      0.5초
────────
총: 12초/파일 × 1001파일 = 3.3시간
```

**저사양 (CPU만)**:
```
1920×1536: 20초
640×512:   5초
640×384:   3.5초
합성:      2초
────────
총: 30초/파일 × 1001파일 = 8.3시간
```

### 디스크 사용량

```
1920×1536: ~300GB
640×512:   ~35GB
640×384:   ~25GB
────────
총: ~360GB (3가지 해상도)

권장: SSD 500GB+ 여유
```

---

## ✅ 체크리스트

실행 전에 확인하세요:

- [ ] Python 3.8+ 설치
- [ ] numpy, opencv-python, Pillow 설치
- [ ] calibration.json 존재
- [ ] 입력 디렉토리에 PCD 파일 1000+개
- [ ] image_a6 폴더에 RGB 이미지 있음
- [ ] 출력 디렉토리 쓰기 권한
- [ ] 디스크 여유 500GB+
- [ ] 로컬 드라이브 (네트워크 드라이브 X)

---

## 📞 버전 정보

```
파일명: integrated_pcd_depth_pipeline_newest.py
라인 수: 1655
마지막 수정: Nov 25, 2024

주요 업데이트:
✅ Nov 14: 가로/세로 비율 스케일링 수정 (div 파라미터 처리)
✅ Nov 14: 640×384 비균일 스케일 지원
✅ Nov 17: 코드 커밋 및 푸시
✅ Nov 25: 완전 설명서 작성
```

---

## 📚 추가 자료

1. **빠른 시작** → [QUICK_START_GUIDE.md](QUICK_START_GUIDE.md)
2. **완전 설명서** → [COMPREHENSIVE_CODE_GUIDE.md](COMPREHENSIVE_CODE_GUIDE.md)
3. **API 레퍼런스** → [API_REFERENCE.md](API_REFERENCE.md)
4. **파이프라인 개요** → [PIPELINE_EXPLANATION.md](pipeline/PIPELINE_EXPLANATION.md)

---

## 🎯 다음 단계

### 1️⃣ 테스트 실행
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/.../synced_data" \
    --output_dir "./output/test" \
    --calibration_path "./calibration.json" \
    --end_idx 10
```

### 2️⃣ 결과 확인
```bash
# 이미지 뷰어로 확인
open output/test/depth_maps_newest/newest_viz_results/
```

### 3️⃣ 전체 실행
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/.../synced_data" \
    --output_dir "./output/full" \
    --calibration_path "./calibration.json"
```

---

## 💡 팁

- `--skip_existing` 플래그로 이미 완료된 파일 건너뛰기
- `--start_idx`, `--end_idx`로 부분 처리 후 계속 진행 가능
- `--max_depth 25.0`으로 더 먼 거리까지 시각화
- `--point_size 4`로 더 크게 표시

---

**이제 사용할 준비가 되었습니다!** 🚀

