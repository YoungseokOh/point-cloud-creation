# ✅ 하드코딩 제거 완료! Print 문제 해결

## 🎯 변경 요약

### 문제점 (수정 전)
```python
# 라인 1407-1408 (문제)
if process_resized:  # ❌ 이것은 구 코드 구조 (bool 예상)
    print(f"[640x384] All 640x384 outputs saved to: {resized_base_dir}")  # ❌ 640x384만 하드코딩
    # resized_base_dir는 정의되지 않음!
```

**문제**:
1. `process_resized`는 이제 dict `{(640,512): True, (640,384): True}`
2. `if process_resized:` 는 항상 True (dict는 non-empty)
3. `resized_base_dir` 변수 범위 오류
4. 640x384만 하드코딩됨

### 해결 (수정 후)
```python
# 라인 1407-1413 (해결)
# [ADD] Print resized resolutions summary
for res_size in resized_resolutions:
    res_name = f"{res_size[0]}x{res_size[1]}"
    if process_resized[res_size]:  # ✅ 각 해상도별 체크
        print(f"[{res_name}] All {res_name} outputs saved to: {resized_dirs[res_size]['base']}")
        # ✅ [640x512] All 640x512 outputs saved to: .../640x512_newest
        # ✅ [640x384] All 640x384 outputs saved to: .../640x384_newest
```

---

## 📊 Print 메시지 전체 흐름

### Phase 1: 초기화 (라인 1070-1104)

```
[640x512] Processing 640x512 resolution outputs
[640x384] Processing 640x384 resolution outputs

[DEBUG] Output base_dir: D:\data\ncdb-cls\...\synced_data
  - newest_pcd: D:\data\ncdb-cls\...\synced_data\newest_pcd
  - newest_depth_maps: D:\data\ncdb-cls\...\synced_data\newest_depth_maps
  - newest_viz_results: D:\data\ncdb-cls\...\synced_data\newest_viz_results
  - newest_colormap: D:\data\ncdb-cls\...\synced_data\newest_colormap
  - newest_synthetic_depth_maps: D:\data\ncdb-cls\...\synced_data\newest_synthetic_depth_maps
  - diff_results: D:\data\ncdb-cls\...\synced_data\diff_results

[DEBUG] 640x512 Output base_dir: D:\data\ncdb-cls\...\synced_data\640x512_newest
[DEBUG] 640x384 Output base_dir: D:\data\ncdb-cls\...\synced_data\640x384_newest
```

**뜻**:
- `[640x512] Processing...` → 640x512_newest 폴더가 없으므로 처리 시작
- `[640x384] Processing...` → 640x384_newest 폴더가 없으므로 처리 시작
- `[DEBUG] ... Output base_dir` → 각 해상도의 기본 폴더 경로 확인

### Phase 2: PCD 파일 처리 (라인 1219-1298)

각 PCD 파일마다 반복:

```
[SAVE] PCD saved (1234 points): newest_pcd\0000000001.pcd
[SAVE] Output PCD saved: newest_pcd\0000000001_closest_line.pcd
[SAVE] Visualization saved: newest_viz_results\0000000001_depth_analysis.png
[SAVE] Colorized depth map saved (JET colormap, from_merged): newest_colormap\0000000001_colorized.png

[SAVE] RGB+Depth visualization (640x512) saved: 0000000001_depth_analysis.png
[640x512] Saved all 640x512 outputs for 0000000001

[SAVE] RGB+Depth visualization (640x384) saved: 0000000001_depth_analysis.png
[640x384] Saved all 640x384 outputs for 0000000001
```

**뜻**:
- `[SAVE]` → 1920×1536 원본 해상도 저장
- `[SAVE] RGB+Depth visualization (640x512)` → 640x512 RGB+depth 저장
- `[640x512] Saved all...` → 이 PCD의 640x512 모든 작업 완료
- `[SAVE] RGB+Depth visualization (640x384)` → 640x384 RGB+depth 저장
- `[640x384] Saved all...` → 이 PCD의 640x384 모든 작업 완료

### Phase 3: 최종 요약 (라인 1407-1413)

```
Processed: 50 files
Failed: 0 files
Output directories:
  - Closest-line PCDs: D:\data\ncdb-cls\...\synced_data\newest_pcd
  - Raw depth maps (16bit): D:\data\ncdb-cls\...\synced_data\newest_depth_maps
  - Analysis plots: D:\data\ncdb-cls\...\synced_data\newest_viz_results
  - Colorized images: D:\data\ncdb-cls\...\synced_data\newest_colormap
  - Diff (merged/synth/orig): D:\data\ncdb-cls\...\synced_data\diff_results

[640x512] All 640x512 outputs saved to: D:\data\ncdb-cls\...\synced_data\640x512_newest
[640x384] All 640x384 outputs saved to: D:\data\ncdb-cls\...\synced_data\640x384_newest
```

**뜻**:
- 마지막에 각 해상도별로 저장 위치 출력
- `[640x512] All 640x512...` → 640x512 모든 작업 완료, 경로 확인
- `[640x384] All 640x384...` → 640x384 모든 작업 완료, 경로 확인

---

## 🔍 Print 메시지 유형 분류

### 1. 초기화 메시지 (라인 1090)
```python
print(f"[{res_name}] Processing {res_name} resolution outputs")
# 또는
print(f"[{res_name}] Skipping - {res_name}_newest already exists")
```

**목적**: 각 해상도의 처리 상태 알림
- ✅ 첫 실행: `Processing...`
- ⏭️ 재실행: `Skipping...`

### 2. 저장 메시지 (라인 1284)
```python
print(f"[SAVE] RGB+Depth visualization ({res_name}) saved: {resized_viz_path.name}")
```

**목적**: 각 파일의 해상도별 시각화 저장 확인
- RGB+Depth 이미지가 저장됨을 알림
- 해상도명 포함 (640x512, 640x384)

### 3. 완료 메시지 (라인 1298)
```python
print(f"[{res_name}] Saved all {res_name} outputs for {stem}")
```

**목적**: 각 PCD 파일의 해상도별 처리 완료 알림
- 한 PCD 파일의 모든 해상도 처리 완료

### 4. DEBUG 메시지 (라인 1104)
```python
print(f"[DEBUG] {res_name} Output base_dir: {resized_dirs[res_size]['base']}")
```

**목적**: 설정 확인 및 문제 진단
- 각 해상도의 기본 폴더 경로 출력
- 폴더 경로 오류 디버깅에 유용

### 5. 최종 요약 메시지 (라인 1412)
```python
print(f"[{res_name}] All {res_name} outputs saved to: {resized_dirs[res_size]['base']}")
```

**목적**: 전체 처리 완료 시 해상도별 결과 경로 확인
- 최종 저장 위치 명시
- 사용자가 결과물 찾기 쉬움

---

## 📋 Print 메시지별 단계 (타임라인)

```
┌─────────────────────────────────────────────────┐
│ 프로그램 시작                                    │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ [라인 1090] 초기화 단계                          │
│                                                │
│ [640x512] Processing 640x512 resolution       │
│ [640x384] Processing 640x384 resolution       │
│                                                │
│ [DEBUG] Output base_dir: ...                   │
│ [DEBUG] 640x512 Output base_dir: ...          │
│ [DEBUG] 640x384 Output base_dir: ...          │
└─────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────┐
│ [라인 1130] PCD 파일 반복 (첫 번째 파일)        │
│                                                │
│ [SAVE] PCD saved ... 0000000001.pcd           │
│ [SAVE] Output PCD saved ...                    │
│ [SAVE] Visualization saved (1920×1536) ...     │
│ [SAVE] Colorized depth map saved ...           │
│                                                │
│ ├─ [라인 1219] 640×512 처리                    │
│ │  [SAVE] RGB+Depth visualization (640x512)    │
│ │  [640x512] Saved all 640x512 outputs         │
│ │                                              │
│ └─ [라인 1219] 640×384 처리                    │
│    [SAVE] RGB+Depth visualization (640x384)    │
│    [640x384] Saved all 640x384 outputs         │
│                                                │
│ ↓ (다음 PCD 파일로 반복)                       │
└─────────────────────────────────────────────────┘
                    ↓
        (50개 PCD 파일 모두 반복)
                    ↓
┌─────────────────────────────────────────────────┐
│ [라인 1407] 최종 요약                           │
│                                                │
│ Processed: 50 files                            │
│ Failed: 0 files                                │
│ Output directories:                            │
│   - Closest-line PCDs: ...                     │
│   - Raw depth maps: ...                        │
│   - Analysis plots: ...                        │
│   - Colorized images: ...                      │
│   - Diff: ...                                  │
│                                                │
│ [640x512] All 640x512 outputs saved to: ...   │
│ [640x384] All 640x384 outputs saved to: ...   │
└─────────────────────────────────────────────────┘
```

---

## ✅ 해결된 문제 목록

### 문제 1: 하드코딩된 해상도명
```python
# ❌ 이전
print(f"[640x384] All 640x384 outputs saved...")

# ✅ 이후
print(f"[{res_name}] All {res_name} outputs saved...")
```

### 문제 2: 범위 오류 (resized_base_dir)
```python
# ❌ 이전 (resized_base_dir 정의 범위 벗어남)
if process_resized:
    print(f"... {resized_base_dir}")

# ✅ 이후 (dict에서 가져옴)
if process_resized[res_size]:
    print(f"... {resized_dirs[res_size]['base']}")
```

### 문제 3: 타입 불일치
```python
# ❌ 이전 (process_resized는 dict이므로 항상 True)
if process_resized:  # dict는 non-empty면 True

# ✅ 이후 (각 해상도별 bool 값 체크)
if process_resized[res_size]:  # 각 해상도의 bool 값
```

### 문제 4: 새 해상도 추가 불가
```python
# ❌ 이전 (새 해상도 추가 시 코드 수정 필요)
if process_resized:
    print(f"[640x384] ...")  # 여기도 수정
    print(f"[640x512] ...")  # 여기도 수정

# ✅ 이후 (자동으로 모든 해상도 처리)
for res_size in resized_resolutions:  # 리스트의 모든 해상도
    print(f"[{res_name}] ...")  # 자동으로 해상도명 적용
```

---

## 🚀 640×512 실행 시 예상 출력

### 첫 실행 (새 폴더 생성)

```
[640x512] Processing 640x512 resolution outputs
[640x384] Processing 640x384 resolution outputs

[DEBUG] Output base_dir: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data
  - newest_pcd: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\newest_pcd
  - newest_depth_maps: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\newest_depth_maps
  - newest_viz_results: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\newest_viz_results
  - newest_colormap: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\newest_colormap
  - newest_synthetic_depth_maps: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\newest_synthetic_depth_maps
  - diff_results: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\diff_results

[DEBUG] 640x512 Output base_dir: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\640x512_newest
[DEBUG] 640x384 Output base_dir: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\640x384_newest

Processing PCD files...
File 1/50: 0000000001.pcd
[SAVE] PCD saved (1234 points): newest_pcd\0000000001.pcd
[SAVE] Output PCD saved: newest_pcd\0000000001_closest_line.pcd
[SAVE] Visualization saved: newest_viz_results\0000000001_depth_analysis.png
[SAVE] Colorized depth map saved (JET colormap, from_merged): newest_colormap\0000000001_colorized.png

[SAVE] RGB+Depth visualization (640x512) saved: 0000000001_depth_analysis.png
[640x512] Saved all 640x512 outputs for 0000000001

[SAVE] RGB+Depth visualization (640x384) saved: 0000000001_depth_analysis.png
[640x384] Saved all 640x384 outputs for 0000000001

... (49 더 많은 PCD 파일) ...

File 50/50: 0000000050.pcd
[SAVE] PCD saved (1234 points): newest_pcd\0000000050.pcd
[SAVE] Output PCD saved: newest_pcd\0000000050_closest_line.pcd
[SAVE] Visualization saved: newest_viz_results\0000000050_depth_analysis.png
[SAVE] Colorized depth map saved (JET colormap, from_merged): newest_colormap\0000000050_colorized.png

[SAVE] RGB+Depth visualization (640x512) saved: 0000000050_depth_analysis.png
[640x512] Saved all 640x512 outputs for 0000000050

[SAVE] RGB+Depth visualization (640x384) saved: 0000000050_depth_analysis.png
[640x384] Saved all 640x384 outputs for 0000000050

Processed: 50 files
Failed: 0 files
Output directories:
  - Closest-line PCDs: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\newest_pcd
  - Raw depth maps (16bit): D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\newest_depth_maps
  - Analysis plots: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\newest_viz_results
  - Colorized images: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\newest_colormap
  - Diff (merged/synth/orig): D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\diff_results

[640x512] All 640x512 outputs saved to: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\640x512_newest
[640x384] All 640x384 outputs saved to: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data\640x384_newest
```

### 재실행 (폴더 이미 있음)

```
[640x512] Skipping - 640x512_newest already exists
[640x384] Skipping - 640x384_newest already exists

[DEBUG] Output base_dir: D:\data\ncdb-cls\ncdb-cls\2025-07-11_15-00-27_410410_A\synced_data

Processing PCD files...
(0 files 처리됨 - 모두 skip)

Processed: 0 files
Failed: 0 files
Output directories:
  ...
```

---

## 🎯 정리

| 단계 | 코드 라인 | 출력 메시지 | 목적 |
|------|---------|-----------|------|
| **초기화** | 1090 | `[640x512] Processing...` | 해상도 처리 시작 알림 |
| **초기화** | 1104 | `[DEBUG] 640x512 Output base_dir...` | 폴더 경로 확인 |
| **처리** | 1284 | `[SAVE] RGB+Depth visualization (640x512)...` | 각 파일의 저장 확인 |
| **완료** | 1298 | `[640x512] Saved all 640x512 outputs...` | 해상도 완료 알림 |
| **최종** | 1412 | `[640x512] All 640x512 outputs saved to...` | 최종 경로 표시 |

**모든 메시지가 동적으로 해상도명을 포함하므로, 새 해상도 추가 시 자동으로 적용됩니다!** 🎉
