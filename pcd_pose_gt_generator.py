#!/usr/bin/env python3
"""
PCD 기반 Pose GT 생성 및 검증 통합 스크립트
사용법: python pcd_pose_gt_generator.py --pcd_paths /path/to/pcd1.pcd /path/to/pcd2.pcd --output_path /path/to/output.json
"""

import argparse
import json
import os
import numpy as np
import open3d as o3d
from typing import List, Dict, Tuple, Optional

class PCDPoseGTGenerator:
    """PCD를 이용한 Pose GT 생성 및 검증 클래스"""
    
    def __init__(self, voxel_sizes: List[float] = [0.05], thresholds: List[float] = [0.1], max_iteration: int = 2000):
        """
        초기화
        Args:
            voxel_sizes: 다운샘플링 voxel 크기 리스트
            thresholds: ICP 거리 임계값 리스트
            max_iteration: ICP 최대 반복 횟수
        """
        self.voxel_sizes = voxel_sizes
        self.thresholds = thresholds
        self.max_iteration = max_iteration
    
    def load_and_preprocess_pcd(self, pcd_path: str, voxel_size: float) -> Optional[o3d.geometry.PointCloud]:
        """PCD 파일 로드 및 전처리"""
        try:
            pcd = o3d.io.read_point_cloud(pcd_path)
            if pcd.is_empty():
                print(f"Warning: Empty PCD file {pcd_path}")
                return None
            
            # 다운샘플링
            pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
            
            # 노이즈 제거
            pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
            
            return pcd
        except Exception as e:
            print(f"Error loading PCD {pcd_path}: {e}")
            return None
    
    def compute_icp_pose(self, pcd1: o3d.geometry.PointCloud, pcd2: o3d.geometry.PointCloud, threshold: float) -> Dict:
        """ICP를 이용한 pose 계산"""
        # 초기 변환 행렬
        trans_init = np.eye(4)
        
        # ICP 수행
        reg_p2p = o3d.pipelines.registration.registration_icp(
            pcd2, pcd1, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=self.max_iteration)
        )
        
        # 변환 행렬에서 pose 추출
        pose_matrix = reg_p2p.transformation
        rotation = pose_matrix[:3, :3]
        translation = pose_matrix[:3, 3]
        
        return {
            'rotation': rotation.tolist(),
            'translation': translation.tolist(),
            'fitness': reg_p2p.fitness,
            'inlier_rmse': reg_p2p.inlier_rmse,
            'transformation_matrix': pose_matrix.tolist()
        }
    
    def compute_reprojection_error(self, pcd1: o3d.geometry.PointCloud, pcd2: o3d.geometry.PointCloud, 
                                  pose_data: Dict, threshold: float) -> Dict:
        """재투영 에러 계산"""
        # 변환 행렬 복원
        pose_matrix = np.array(pose_data['transformation_matrix'])
        
        # PCD2를 변환
        pcd2_transformed = pcd2.transform(pose_matrix)
        
        # 최근접점 거리 계산
        distances = np.asarray(pcd2_transformed.compute_point_cloud_distance(pcd1))
        
        # 통계 계산
        mean_error = np.mean(distances)
        max_error = np.max(distances)
        median_error = np.median(distances)
        std_error = np.std(distances)
        
        # 유효한 포인트 비율 (임계값 이내)
        valid_ratio = np.sum(distances < threshold) / len(distances)
        
        return {
            'mean_reprojection_error': mean_error,
            'max_reprojection_error': max_error,
            'median_reprojection_error': median_error,
            'std_reprojection_error': std_error,
            'valid_point_ratio': valid_ratio,
            'total_points': len(distances)
        }
    
    def process_pcd_pair(self, pcd_path1: str, pcd_path2: str) -> Optional[Dict]:
        """단일 PCD 쌍 처리 (여러 파라미터 조합 시도)"""
        print(f"Processing PCD pair: {os.path.basename(pcd_path1)} <-> {os.path.basename(pcd_path2)}")
        
        best_result = None
        max_fitness = -1.0

        for voxel_size in self.voxel_sizes:
            # PCD 로드 및 전처리 (현재 voxel_size 사용)
            pcd1_preprocessed = self.load_and_preprocess_pcd(pcd_path1, voxel_size)
            pcd2_preprocessed = self.load_and_preprocess_pcd(pcd_path2, voxel_size)
            
            if pcd1_preprocessed is None or pcd2_preprocessed is None:
                print(f"Warning: Skipping pair due to empty or failed PCD load/preprocess for voxel_size={voxel_size}")
                continue

            for threshold in self.thresholds:
                print(f"  Trying voxel_size={voxel_size:.3f}, threshold={threshold:.3f}")
                
                # ICP로 pose 계산 (현재 threshold 사용)
                pose_data = self.compute_icp_pose(pcd1_preprocessed, pcd2_preprocessed, threshold)
                
                if pose_data['fitness'] > max_fitness:
                    max_fitness = pose_data['fitness']
                    
                    # 재투영 에러 계산
                    error_data = self.compute_reprojection_error(pcd1_preprocessed, pcd2_preprocessed, pose_data, threshold)
                    
                    # 결과 통합
                    best_result = {
                        'pcd_pair': [os.path.basename(pcd_path1), os.path.basename(pcd_path2)],
                        'pcd_paths': [pcd_path1, pcd_path2],
                        'pose': pose_data,
                        'reprojection_error': error_data,
                        'is_valid_gt': bool(error_data['mean_reprojection_error'] < threshold), # 현재 threshold 사용
                        'best_voxel_size': voxel_size,
                        'best_threshold': threshold
                    }
        
        if best_result:        
            print(f"  Best result found with voxel_size={best_result['best_voxel_size']:.3f}, threshold={best_result['best_threshold']:.3f}")
            print(f"  Fitness: {best_result['pose']['fitness']:.4f}, Mean Error: {best_result['reprojection_error']['mean_reprojection_error']:.4f}")
        else:
            print(f"  No valid ICP result found for pair {os.path.basename(pcd_path1)} <-> {os.path.basename(pcd_path2)}")

        return best_result
    
    def process_multiple_pairs(self, pcd_paths: List[str]) -> List[Dict]:
        """여러 PCD 쌍 처리"""
        results = []
        
        # 2개씩 짝지어 처리
        for i in range(0, len(pcd_paths), 2):
            if i + 1 < len(pcd_paths):
                result = self.process_pcd_pair(pcd_paths[i], pcd_paths[i+1])
                if result:
                    results.append(result)
        
        return results
    
    def save_results(self, results: List[Dict], output_path: str):
        """결과를 JSON 파일로 저장"""
        with open(output_path, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"Results saved to {output_path}")
    
    def print_summary(self, results: List[Dict]):
        """결과 요약 출력"""
        if not results:
            print("No valid results to summarize")
            return
        
        print("\n" + "="*60)
        print("POSE GT GENERATION SUMMARY")
        print("="*60)
        
        valid_count = sum(1 for r in results if r['is_valid_gt'])
        total_pairs = len(results)
        
        print(f"Total PCD pairs processed: {total_pairs}")
        print(f"Valid pose GT pairs: {valid_count} ({valid_count/total_pairs*100:.1f}%)")
        
        if results:
            errors = [r['reprojection_error']['mean_reprojection_error'] for r in results]
            print(f"Mean reprojection error: {np.mean(errors):.4f}")
            print(f"Min reprojection error: {np.min(errors):.4f}")
            print(f"Max reprojection error: {np.max(errors):.4f}")
        
        print("\nDetailed Results:")
        for i, result in enumerate(results):
            status = "[VALID]" if result['is_valid_gt'] else "[INVALID]"
            error = result['reprojection_error']['mean_reprojection_error']
            fitness = result['pose']['fitness']
            best_voxel_size = result.get('best_voxel_size', 'N/A')
            best_threshold = result.get('best_threshold', 'N/A')
            print(f"  {i+1}. {result['pcd_pair'][0]} <-> {result['pcd_pair'][1]}")
            print(f"      Status: {status} | Error: {error:.4f} | Fitness: {fitness:.4f}")
            print(f"      Best Params: Voxel={best_voxel_size:.3f}, Threshold={best_threshold:.3f}")


def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description="PCD 기반 Pose GT 생성 및 검증")
    parser.add_argument('--pcd_paths', nargs='+', required=True, 
                       help='PCD 파일 경로들 (짝수 개)')
    parser.add_argument('--output_path', type=str, default='output/pcd_pose_gt_results.json',
                       help='출력 JSON 파일 경로')
    parser.add_argument('--voxel_sizes', nargs='+', type=float, default=[0.05], 
                       help='다운샘플링 voxel 크기 리스트 (공백으로 구분)')
    parser.add_argument('--thresholds', nargs='+', type=float, default=[0.1], 
                       help='ICP 거리 임계값 리스트 (공백으로 구분)')
    parser.add_argument('--max_iteration', type=int, default=2000,
                       help='ICP 최대 반복 횟수')
    
    args = parser.parse_args()
    
    # PCD 파일 존재 확인
    for pcd_path in args.pcd_paths:
        if not os.path.exists(pcd_path):
            print(f"Error: PCD file not found: {pcd_path}")
            return
    
    if len(args.pcd_paths) % 2 != 0:
        print("Warning: Odd number of PCD files. Last file will be ignored.")
    
    # Pose GT 생성기 초기화
    generator = PCDPoseGTGenerator(
        voxel_sizes=args.voxel_sizes,
        thresholds=args.thresholds,
        max_iteration=args.max_iteration
    )
    
    # PCD 쌍 처리
    results = generator.process_multiple_pairs(args.pcd_paths)
    
    # 결과 저장
    generator.save_results(results, args.output_path)
    
    # 요약 출력
    generator.print_summary(results)


if __name__ == "__main__":
    main()