# 📚 Documentation Hub

완전한 파이프라인 설명서 및 API 레퍼런스

---

## ⭐ 신규 가이드 (Nov 25, 2024)

### 🚀 `README_INTEGRATED_PIPELINE.md` - **여기서 시작하세요!**
**핵심 파이프라인의 완전한 소개**
- 파이프라인 개요 및 주요 특징
- 명령어 사용법 (기본 → 고급)
- 출력 파일 설명
- 핵심 개념 설명

### 📖 `QUICK_START_GUIDE.md` - **5분 안에 실행하기**
**처음 사용자를 위한 가이드**
- 환경 설정 체크리스트
- 테스트 실행 (10개 파일)
- 실전 명령어 예제 7가지
- 결과 확인 방법
- 일반적인 문제 해결

### 📘 `COMPREHENSIVE_CODE_GUIDE.md` - **완전한 코드 설명**
**코드 동작 원리를 완벽히 이해하려는 사용자를 위한 문서**
- 파이프라인 전체 흐름도
- VADAS Fisheye 카메라 모델 (11-parameter intrinsic)
- 스케일 적용 방식 (가로/세로 비율 유지)
- 깊이맵 저장 형식 (KITTI uint16)
- 합성 포인트 생성 (C-circle 패턴)
- 주요 함수 상세 설명
- 출력 파일 해석
- 전체 명령어 옵션 (20+)
- 성능 최적화 팁

### 📚 `API_REFERENCE.md` - **프로그래머용 API 문서**
**소프트웨어 개발자를 위한 상세 레퍼런스**
- `VADASFisheyeCameraModel` 클래스 API
  - 초기화, scale_intrinsics(), project_point()
  - 투영 공식 (상세)
  - 속성 및 메서드

- `CalibrationDB` 클래스 API
- `LidarCameraProjector` 클래스 API
  - project_cloud_to_depth_map_with_labels() 상세
  - 동작 흐름 다이어그램

- 유틸리티 함수
  - PCD 파일 처리
  - 깊이맵 저장/로드
  - RGB+Depth 시각화
  - Colormap 생성

- 데이터 형식 스펙
- 통합 예제 (완전한 워크플로우)
- 성능 최적화

---

## 📂 기존 문서 구조

### 📘 `/pipeline`
파이프라인 아키텍처 및 구현 세부사항
- `PIPELINE_EXPLANATION.md` - 파이프라인 워크플로우 설명
- `PIPELINE_QUICK_REFERENCE.md` - 빠른 참조
- `README_PIPELINE.md` - 파이프라인 개요

### 📐 `/resolution_support`
다중 해상도 지원 및 스케일링 구현
- `COMPLETE_640x512_GUIDE.md` - 640×512 완전 가이드
- `HOW_TO_USE_640x512.md` - 640×512 사용 가이드
- `DYNAMIC_RESOLUTION_SUPPORT.md` - 동적 해상도 지원
- `SCALING_DIV_FIX.md` - 스케일링 및 div 파라미터 수정 (Nov 14)
- `SCALING_CORRECTION.md` - 스케일링 보정 상세

### 🔍 `/analysis`
코드 분석 및 리팩토링 문서
- `RGB_RESIZE_ANALYSIS.md` - RGB 이미지 리사이징 분석
- `VISUALIZATION_ANALYSIS.md` - 시각화 접근 분석
- `PRINT_MESSAGE_ANALYSIS.md` - 출력 메시지 표준화
- `FOLDER_STRUCTURE_ANALYSIS.md` - 폴더 구조 분석

### 📖 `/guides`
사용자 가이드 및 참고 자료
- `CODE_CHANGE_SUMMARY.md` - 코드 변경사항 요약
- `instruction.md` - 일반 지침
- `reference_code_paths.md` - 참고 코드 경로

### 📦 `/archive`
역사적 문서 및 프롬프트
- `GEMINI.md` - Gemini 관련 문서
- `synth_lidar_stage2-4_prompt.md` - 합성 LiDAR 프롬프트

---

## 🎯 어떤 가이드를 읽어야 할까?

### 1. **"지금 바로 파이프라인을 실행하고 싶어요"**
→ [`QUICK_START_GUIDE.md`](QUICK_START_GUIDE.md) (5분)

```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/ncdb-cls/.../synced_data" \
    --output_dir "./output/test" \
    --calibration_path "./calibration.json" \
    --end_idx 10
```

### 2. **"파이프라인이 어떻게 작동하는지 완벽히 이해하고 싶어요"**
→ [`COMPREHENSIVE_CODE_GUIDE.md`](COMPREHENSIVE_CODE_GUIDE.md) (30분)

포함 내용:
- 파이프라인 전체 흐름도
- VADAS Fisheye 카메라 모델 설명
- Nov 14 스케일링 버그 수정 (div 파라미터)
- 모든 명령어 옵션 상세 설명
- 출력 파일 해석

### 3. **"직접 코드를 작성하거나 API를 사용하고 싶어요"**
→ [`API_REFERENCE.md`](API_REFERENCE.md) (상세 레퍼런스)

포함 내용:
- 각 클래스의 초기화, 메서드, 파라미터
- 투영 공식 상세
- 데이터 형식 스펙
- 통합 예제 코드

### 4. **"빠른 명령어 예제를 보고 싶어요"**
→ [`README_INTEGRATED_PIPELINE.md`](README_INTEGRATED_PIPELINE.md) (명령어 요약)

5가지 해상도, 성능, 체크리스트 포함

---

## 📊 핵심 기술

### VADAS Fisheye 카메라 모델
- **11-parameter intrinsic**: k[0:7] (다항식), s (크기), div (왜곡), ux, uy (주점)
- **중요**: div 파라미터는 절대 스케일링하면 안됨 (Nov 14 발견)
- **스케일 적용**: scale_x, scale_y를 최종 픽셀 좌표에만 적용

### 다중 해상도 지원
- **1920×1536**: 원본 해상도
- **640×512**: 균일 스케일 (0.333×0.333)
- **640×384**: 비균일 스케일 (0.333×0.250) ← 종횡비 변경
- **사용자 정의**: 원하는 해상도 추가 가능

### 출력 형식
- **깊이맵**: uint16 PNG (KITTI 포맷: value/256 = 미터)
- **RGB+Depth**: 깊이값을 색상으로 표시한 오버레이
- **Colormap**: Jet colormap으로 시각화
- **PCD**: Binary 포인트 클라우드

---

## ✅ 빠른 체크리스트

실행 전:
- [ ] Python 3.8+ 설치
- [ ] numpy, opencv-python, Pillow 설치
- [ ] calibration.json 준비
- [ ] 입력 데이터 확인 (PCD 1000+개, RGB 이미지)
- [ ] 디스크 여유 500GB+

---

## 🚀 시작하기

### 단계 1: 가이드 선택
1. **빠르게 시작** → [`QUICK_START_GUIDE.md`](QUICK_START_GUIDE.md)
2. **완전히 이해** → [`COMPREHENSIVE_CODE_GUIDE.md`](COMPREHENSIVE_CODE_GUIDE.md)
3. **API 사용** → [`API_REFERENCE.md`](API_REFERENCE.md)

### 단계 2: 테스트 실행
```bash
python integrated_pcd_depth_pipeline_newest.py \
    --input_dir "D:/data/..." \
    --output_dir "./output/test" \
    --calibration_path "./calibration.json" \
    --end_idx 10
```

### 단계 3: 결과 확인
```bash
# 생성된 이미지 확인
open output/test/depth_maps_newest/newest_viz_results/
```

---

## 📈 성능 참고값

**처리 시간** (1파일당):
- 고사양 (GTX 1080+): ~12초 → 3.3시간 (1001파일)
- 저사양 (CPU): ~30초 → 8.3시간 (1001파일)

**디스크 사용량**:
- 3가지 해상도: ~360GB
- 추천: SSD 500GB+ 여유

---

## 📞 문제 해결

| 문제 | 해결책 |
|------|--------|
| "No PCD files found" | 입력 경로 확인 |
| "Depth map is all black" | calibration.json 검증 |
| "Out of memory" | `--batch_size 5 --num_workers 1` |
| "Encoding error" | 자동 처리 (UTF-8) |

자세한 문제 해결은 [`QUICK_START_GUIDE.md`](QUICK_START_GUIDE.md) 참조

---

**모든 가이드는 Nov 25, 2024에 작성되었습니다.** ✨
