# Synthetic LiDAR (Stage 2–4) – Coding Agent Prompt

이 문서는 **Stage 2–4 (Intrinsics 샘플링 → Depth→PointCloud & Mesh 재구성 → LiDAR 레이캐스팅)**를 구현·통합하기 위한 **단일 프롬프트**입니다.  
전제: 1단계(모노큘러 dense depth `D_syn` 생성)는 이미 구현되어 있습니다.

---

## ROLE
너는 Python으로 컴퓨터비전 파이프라인을 구현하는 시니어 엔지니어다. 이미 구현된 1단계(모노큘러 dense depth `D_syn` 생성)를 **그대로 사용**하고, 여기에 **2) 카메라 내부파라미터 샘플링 → 3) 3D 포인트화 & 메쉬 재구성 → 4) LiDAR 레이캐스팅 시뮬레이션**을 **확장 모듈**로 추가한다.  
기존 코드에 통합 시 **변경/추가 라인은 `# [ADDED]`, `# [MOD]`**로 주석을 단다. 새 파일은 깔끔한 모듈/CLI 구조로 만든다.

---

## OBJECTIVE
입력
- RGB 이미지 크기 `W × H`
- 1단계에서 저장된 **dense depth `D_syn`** (미터 단위, NaN/0 무효 허용)

출력
1. 샘플링된 **내부파라미터 `K`**
2. `D_syn`으로부터 복원한 **point cloud `P`**
3. `P`로 만든 **연속 메쉬 `M`** (`.ply`)
4. `M`에 LiDAR 레이캐스팅한 **희소 depth 맵 `D_simu`** 및 **히트 포인트(3D)**
5. 시각화 산출물(컬러맵, 오버레이, 2×2 모자이크)

---

## IMPLEMENTATION REQUIREMENTS

### A. 패키지 & 파일 구조
의존성: `numpy`, `opencv-python`, `open3d`, `trimesh`, `pyembree`(옵션), `matplotlib`, `tqdm`. GPU가 있으면 `pytorch3d`로 레이캐스팅 가속 가능(선택).

파일 구조:
```
synthetic_lidar/
  __init__.py
  intrinsics.py      # 2) K 샘플링
  depth_to_mesh.py   # 3) 포인트화 & 메쉬 재구성
  raycast.py         # 4) 레이캐스팅
  viz.py             # 시각화(컬러맵, 오버레이, 모자이크)
  cli.py             # CLI 엔트리포인트
  config.yaml        # 기본 파라미터(샘플 값 포함)
run_synth_lidar.py   # CLI 래퍼
```

### B. 2) Intrinsics 샘플링 (`intrinsics.py`)
반환해야 하는 카메라 내부파라미터:
\[
K = \begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
\]

함수 시그니처(예시):
```python
def sample_intrinsics(W:int, H:int,
                      fovx_range=(60.0, 120.0),  # deg
                      fovy_range=(45.0, 100.0),  # deg
                      cx_jitter_ratio=0.05,
                      cy_jitter_ratio=0.05,
                      keep_square_pixels=True,
                      rng_seed=None) -> np.ndarray:
    '''
    Return K (3x3). fx = 0.5*W / tan(0.5*FOVx), fy = 0.5*H / tan(0.5*FOVy).
    cx = W/2 + U[-cx_jitter_ratio*W, +cx_jitter_ratio*W]
    cy = H/2 + U[-cy_jitter_ratio*H, +cy_jitter_ratio*H]
    If keep_square_pixels: set fy=fx*H/W after FOVx sampling (override FOVy).
    Clamp fx,fy to [min(W,H)*0.3, max(W,H)*3.0].
    '''
```

### C. 3) 포인트화 & 메쉬 (`depth_to_mesh.py`)
포인트화:
```python
def depth_to_points(D: np.ndarray, K: np.ndarray, mask_valid=None,
                    stride:int=1, z_clip=(0.2, 200.0)) -> np.ndarray:
    '''
    Back-project (u,v,z) -> (X,Y,Z).
    X = (u-cx)/fx * Z, Y = (v-cy)/fy * Z, Z = depth.
    Ignore invalids (<=0, NaN) and outside z_clip.
    Return Nx3 float32 array in camera coords.
    '''
```

메쉬 재구성(기본: Open3D Poisson, 대안: Ball Pivoting):
```python
def reconstruct_mesh(points: np.ndarray,
                     poisson_depth:int=10,
                     remove_radius=0.05,
                     remove_min_neighbors=5,
                     simplify_target_faces:int=200_000):
    '''
    1) build o3d.geometry.PointCloud + estimate_normals
    2) o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(depth=poisson_depth)
    3) crop by density / remove outliers
    4) mesh.simplify_quadric_decimation(target_number_of_triangles=simplify_target_faces)
    return mesh (o3d TriangleMesh)
    '''
```

저장 유틸:
```python
def save_mesh_ply(mesh, path: str) -> None:
    ...
```

### D. 4) LiDAR 레이캐스팅 (`raycast.py`)
설정 구조체:
```python
from dataclasses import dataclass
from typing import Tuple

@dataclass
class LidarConfig:
    n_channels:int = 32
    h_res:int = 2048                 # points per revolution
    v_fov_deg:Tuple[float,float]=(-25.0, 15.0)  # [min,max]
    h_fov_deg:Tuple[float,float]=( -180.0, 180.0)
    origin:Tuple[float,float,float]=(0.0,0.0,0.0) # camera 좌표계 원점 가정
    noise_std:float = 0.0            # optional range noise
    max_range:float = 120.0
```

레이 생성(각도 격자 → 방향 벡터):
- 수평각 `θ` = `linspace(h_fov[0], h_fov[1], h_res)`
- 수직각 `φ` = `linspace(v_fov[0], v_fov[1], n_channels)`
- 단위방향: `d = [cosφ cosθ, sinφ, cosφ sinθ]`

교차 엔진:
- 기본: `trimesh` + `pyembree` (있으면 자동 사용)
- 옵션: `pytorch3d.ops.ray_mesh_intersector` (CUDA 가속)

반환:
```python
def raycast_mesh(mesh, lidar_cfg:LidarConfig):
    '''
    Returns:
      depth_map: (n_channels, h_res) float32, meters (0 or NaN if no hit or >max_range)
      hits_xyz: (n_channels, h_res, 3) float32, NaN where miss
      hits_mask: bool mask
    '''
```

### E. 시각화 (`viz.py`)
- `save_depth_colormap(depth, out_png)` — matplotlib로 컬러맵 이미지 저장  
- `overlay_sparse_on_rgb(rgb, hits_xyz, K, out_png)` — 히트 포인트를 K로 project해서 RGB에 오버레이  
- `make_mosaic(rgb, depth_dense, depth_sparse_cm, overlay, out_png)` — 2×2 모자이크

### F. CLI (`cli.py` / `run_synth_lidar.py`)
사용 예:
```bash
python run_synth_lidar.py   --rgb path/to/image.png   --dense-depth path/to/D_syn.exr   --out-dir outputs/scene_0001   --config synthetic_lidar/config.yaml   --seed 42
```

동작:
1) 입력 로드(RGB, `D_syn`); 크기 `W,H` 추출  
2) **K 샘플링** → 저장(`K.npy`)  
3) **depth→points**(stride는 config로) → **mesh 재구성** → `mesh.ply` 저장  
4) **레이캐스팅** → `D_simu.npy`, `hits_xyz.npy`, `mask.npy` 저장  
5) 시각화 PNG 3종 + 모자이크 1종 저장  
6) 로그/파라미터 `meta.json` 기록

`config.yaml` 기본값(참고):
```yaml
intrinsics:
  fovx_range: [70.0, 110.0]
  fovy_range: [55.0, 90.0]
  cx_jitter_ratio: 0.03
  cy_jitter_ratio: 0.03
  keep_square_pixels: true

depth_to_mesh:
  stride: 2
  z_clip: [0.2, 120.0]
  poisson_depth: 10
  remove_radius: 0.05
  remove_min_neighbors: 5
  simplify_target_faces: 200000

lidar:
  n_channels: 32
  h_res: 2048
  v_fov_deg: [-25.0, 15.0]
  h_fov_deg: [-180.0, 180.0]
  noise_std: 0.01
  max_range: 120.0
```

### G. 검증 & 유닛 테스트(필수)
- **평면 테스트**: Z=10m 평면 `D_syn`을 합성 → 포인트/메쉬/레이캐스팅 후, depth 분포가 `10±0.1 m`에 집중하는지 확인.  
- **히트율 테스트**: v_fov를 좁게 주면 miss 증가, 넓히면 hit 증가하는 단조성 체크.  
- **투영 일관성**: `overlay_sparse_on_rgb` 사용 시, 히트 포인트가 이미지 바운드 내 비율 ≥90%.

### H. 성능/안정성
- 대규모 이미지에서 Poisson이 무거울 수 있으므로 `stride`와 `simplify_target_faces`로 제어.  
- 레이캐스팅은 `pyembree`가 있으면 자동 사용(없으면 CPU fallback).  
- 모든 저장물은 재현 위해 `meta.json`에 파라미터/시드 기록.

---

## DELIVERABLES
1) 위 파일 구조의 모듈과 CLI, 동작 예시 로그.  
2) 샘플 실행 스크립트/명령과 산출 PNG 4종, `.npy`/`.ply`/`K.npy`/`meta.json`.  
3) 통합 시 변경된 기존 파일에 `# [ADDED]`/`# [MOD]` 주석.  
4) 간단한 `README.md` (설치, 실행, 파라미터 설명, 한계와 TODO).

---

## ACCEPTANCE CRITERIA
- `run_synth_lidar.py` 단일 명령으로 2–4 단계 전 과정이 재현되고, 출력 폴더에  
  - `K.npy`, `mesh.ply`, `D_simu.npy`, `hits_xyz.npy`, `mask.npy`,  
  - `depth_dense_cm.png`, `depth_sparse_cm.png`, `overlay.png`, `mosaic_2x2.png`, `meta.json`  
  가 생성되어야 한다.
- `pytest` 3개 테스트 모두 통과.
- 코드 전반에 타입힌트/도크스트링/에러 처리 포함.

---

## Notes
- `D_syn`의 스케일은 상대값일 수 있음. 본 단계에서는 **절대 스케일 보정 없이** 상대 기하만 학습·시뮬 목적.  
- 메쉬 과평활화/경계 아티팩트는 `poisson_depth`, outlier 제거, simplify 파라미터로 튜닝.
