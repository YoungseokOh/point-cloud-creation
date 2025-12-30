"""
LiDAR → Camera 변환 행렬 재계산 및 검증

다양한 조합을 테스트:
1. Extrinsic @ LiDAR_to_World
2. LiDAR_to_World @ Extrinsic
3. inv(Extrinsic) @ LiDAR_to_World
4. Extrinsic @ inv(LiDAR_to_World)
5. 등등...
"""

import numpy as np
import cv2
import math
from pathlib import Path

# =============================================================================
# 원본 데이터
# =============================================================================

# LiDAR → World (v3)
LIDAR_TO_WORLD_v3 = np.array([
    [-0.99856,  -0.00901632,  -0.052883,   0.0539789],
    [0.00872567,  -0.999946,  0.00572437,  0.0837919],
    [-0.0529318, 0.00525468,   0.998584,   0.737086],
    [0.,         0.,          0.,           1.       ]
])

# Extrinsic (Rodrigues 형식: tx, ty, tz, rx, ry, rz)
EXTRINSIC_RODRIGUES = [0.119933, -0.129544, -0.54216, -0.0333289, -0.166123, -0.0830659]

# 기존 DEFAULT_LIDAR_TO_CAM_v3 (참고용)
EXISTING_LIDAR_TO_CAM_v3 = np.array([
    [ 0.98280272,  0.08533396, -0.16375873,  0.119933  ],
    [-0.07981367,  0.99600649,  0.04001059, -0.129544  ],
    [ 0.16651902, -0.02625233,  0.98568871, -0.54216   ],
    [ 0.,          0.,          0.,          1.        ]
])


def rodrigues_to_matrix(rodrigues_vec):
    """Rodrigues 벡터를 4x4 변환 행렬로 변환"""
    tvec = np.array(rodrigues_vec[:3], dtype=np.float64)
    rvec = np.array(rodrigues_vec[3:6], dtype=np.float64)
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec
    return T


def print_matrix(name, mat):
    print(f"\n{name}:")
    print(mat)


def analyze_rotation(R):
    """회전 행렬 분석"""
    # 회전 축과 각도 추출
    trace = np.trace(R[:3, :3])
    angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
    print(f"  회전 각도: {np.degrees(angle):.2f}°")
    
    # 주요 방향 확인
    print(f"  X축 방향: {R[:3, 0]}")
    print(f"  Y축 방향: {R[:3, 1]}")
    print(f"  Z축 방향: {R[:3, 2]}")
    print(f"  Translation: {R[:3, 3]}")


def main():
    print("=" * 70)
    print("LiDAR → Camera 변환 행렬 재계산")
    print("=" * 70)
    
    # Extrinsic 행렬 계산
    EXTRINSIC = rodrigues_to_matrix(EXTRINSIC_RODRIGUES)
    
    print("\n[1] 원본 데이터")
    print_matrix("LIDAR_TO_WORLD_v3", LIDAR_TO_WORLD_v3)
    print("\n분석:")
    analyze_rotation(LIDAR_TO_WORLD_v3)
    
    print_matrix("EXTRINSIC (World → Camera)", EXTRINSIC)
    print("\n분석:")
    analyze_rotation(EXTRINSIC)
    
    print_matrix("기존 LIDAR_TO_CAM_v3", EXISTING_LIDAR_TO_CAM_v3)
    print("\n분석:")
    analyze_rotation(EXISTING_LIDAR_TO_CAM_v3)
    
    # =============================================================================
    # 다양한 조합 테스트
    # =============================================================================
    
    print("\n" + "=" * 70)
    print("[2] 다양한 조합 테스트")
    print("=" * 70)
    
    combinations = {}
    
    # 1. Extrinsic @ LiDAR_to_World
    combinations["1. Extrinsic @ L2W"] = EXTRINSIC @ LIDAR_TO_WORLD_v3
    
    # 2. LiDAR_to_World @ Extrinsic
    combinations["2. L2W @ Extrinsic"] = LIDAR_TO_WORLD_v3 @ EXTRINSIC
    
    # 3. inv(Extrinsic) @ LiDAR_to_World
    combinations["3. inv(Ext) @ L2W"] = np.linalg.inv(EXTRINSIC) @ LIDAR_TO_WORLD_v3
    
    # 4. LiDAR_to_World @ inv(Extrinsic)
    combinations["4. L2W @ inv(Ext)"] = LIDAR_TO_WORLD_v3 @ np.linalg.inv(EXTRINSIC)
    
    # 5. Extrinsic @ inv(LiDAR_to_World)
    combinations["5. Ext @ inv(L2W)"] = EXTRINSIC @ np.linalg.inv(LIDAR_TO_WORLD_v3)
    
    # 6. inv(LiDAR_to_World) @ Extrinsic
    combinations["6. inv(L2W) @ Ext"] = np.linalg.inv(LIDAR_TO_WORLD_v3) @ EXTRINSIC
    
    # 7. inv(Extrinsic @ LiDAR_to_World)
    combinations["7. inv(Ext @ L2W)"] = np.linalg.inv(EXTRINSIC @ LIDAR_TO_WORLD_v3)
    
    # 8. inv(LiDAR_to_World @ Extrinsic)
    combinations["8. inv(L2W @ Ext)"] = np.linalg.inv(LIDAR_TO_WORLD_v3 @ EXTRINSIC)
    
    # 9. Extrinsic만 (LiDAR = World 좌표계 가정)
    combinations["9. Extrinsic only"] = EXTRINSIC
    
    # 10. inv(Extrinsic)만
    combinations["10. inv(Extrinsic)"] = np.linalg.inv(EXTRINSIC)
    
    for name, mat in combinations.items():
        print_matrix(name, mat)
        
        # 기존 v3와 비교
        diff = np.abs(mat - EXISTING_LIDAR_TO_CAM_v3).max()
        print(f"  ↳ 기존 v3와 최대 차이: {diff:.6f}")
    
    # =============================================================================
    # 기존 v3 역분석
    # =============================================================================
    
    print("\n" + "=" * 70)
    print("[3] 기존 LIDAR_TO_CAM_v3 역분석")
    print("=" * 70)
    
    # 기존 v3의 translation이 extrinsic의 translation과 동일
    print("\n기존 v3 translation:", EXISTING_LIDAR_TO_CAM_v3[:3, 3])
    print("Extrinsic translation:", EXTRINSIC[:3, 3])
    print("→ Translation이 동일! Extrinsic의 rotation만 사용된 것 같음")
    
    # 기존 v3의 rotation 부분만 추출
    R_v3 = EXISTING_LIDAR_TO_CAM_v3[:3, :3]
    R_ext = EXTRINSIC[:3, :3]
    
    print("\n기존 v3 Rotation:")
    print(R_v3)
    
    print("\nExtrinsic Rotation:")
    print(R_ext)
    
    # R_v3 = R_ext @ R_something?
    # R_something = inv(R_ext) @ R_v3
    R_something = np.linalg.inv(R_ext) @ R_v3
    print("\nR_something = inv(R_ext) @ R_v3:")
    print(R_something)
    
    # 또는 R_v3 = R_something @ R_ext?
    R_something2 = R_v3 @ np.linalg.inv(R_ext)
    print("\nR_something2 = R_v3 @ inv(R_ext):")
    print(R_something2)
    
    # L2W의 rotation과 비교
    R_l2w = LIDAR_TO_WORLD_v3[:3, :3]
    print("\nL2W Rotation:")
    print(R_l2w)
    
    print("\nR_something과 L2W 차이:")
    print(np.abs(R_something - R_l2w).max())
    
    print("\nR_something과 inv(L2W) 차이:")
    print(np.abs(R_something - np.linalg.inv(LIDAR_TO_WORLD_v3)[:3, :3]).max())
    
    print("\nR_something2와 L2W 차이:")
    print(np.abs(R_something2 - R_l2w).max())
    
    # =============================================================================
    # 가설: LIDAR_TO_CAM_v3 = Extrinsic (rotation만) with original translation
    # =============================================================================
    
    print("\n" + "=" * 70)
    print("[4] 새로운 가설 테스트")
    print("=" * 70)
    
    # 가설 1: 단순히 Extrinsic의 회전에 추가 회전 적용
    # LiDAR 좌표계 → 카메라 좌표계 변환 (180도 회전 등)
    
    # Z축 180도 회전
    Rz_180 = np.array([
        [-1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float64)
    
    # X축 180도 회전
    Rx_180 = np.array([
        [1, 0, 0, 0],
        [0, -1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float64)
    
    # Y축 180도 회전
    Ry_180 = np.array([
        [-1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, -1, 0],
        [0, 0, 0, 1]
    ], dtype=np.float64)
    
    print("\n테스트: Extrinsic @ Rz_180")
    test1 = EXTRINSIC @ Rz_180
    print(test1)
    print(f"  차이: {np.abs(test1 - EXISTING_LIDAR_TO_CAM_v3).max():.6f}")
    
    print("\n테스트: Rz_180 @ Extrinsic")
    test2 = Rz_180 @ EXTRINSIC
    print(test2)
    print(f"  차이: {np.abs(test2 - EXISTING_LIDAR_TO_CAM_v3).max():.6f}")
    
    print("\n테스트: Extrinsic @ Rx_180")
    test3 = EXTRINSIC @ Rx_180
    print(test3)
    print(f"  차이: {np.abs(test3 - EXISTING_LIDAR_TO_CAM_v3).max():.6f}")
    
    print("\n테스트: Rx_180 @ Extrinsic")
    test4 = Rx_180 @ EXTRINSIC
    print(test4)
    print(f"  차이: {np.abs(test4 - EXISTING_LIDAR_TO_CAM_v3).max():.6f}")
    
    # =============================================================================
    # 결론
    # =============================================================================
    
    print("\n" + "=" * 70)
    print("[5] 결론 및 권장 사항")
    print("=" * 70)
    
    # 가장 차이가 작은 조합 찾기
    min_diff = float('inf')
    best_name = ""
    for name, mat in combinations.items():
        diff = np.abs(mat - EXISTING_LIDAR_TO_CAM_v3).max()
        if diff < min_diff:
            min_diff = diff
            best_name = name
    
    print(f"\n기존 v3와 가장 유사한 조합: {best_name}")
    print(f"최대 차이: {min_diff:.6f}")
    
    if min_diff > 0.1:
        print("\n⚠️ 기존 v3는 단순한 행렬 곱으로 계산된 것이 아님!")
        print("   Translation이 Extrinsic과 동일하므로, 별도의 방식으로 계산된 것으로 보임")


if __name__ == "__main__":
    main()
