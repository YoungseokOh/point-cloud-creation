#!/usr/bin/env python3
"""
Image Resizer for image_a6 folders
Finds image_a6 folders in the given parent directory and resizes all images to 640x384,
overwriting the original files.
"""

import cv2
import numpy as np
from pathlib import Path
from typing import List, Tuple
import argparse
from tqdm import tqdm


def find_image_a6_folders(parent_folder: Path) -> List[Path]:
    """
    주어진 부모 폴더에서 image_a6 폴더들을 재귀적으로 찾습니다.
    
    Args:
        parent_folder: 검색할 부모 폴더
        
    Returns:
        찾은 image_a6 폴더들의 경로 리스트
    """
    image_a6_folders = []
    
    # 현재 폴더에서 image_a6 찾기
    direct_image_a6 = parent_folder / "image_a6"
    if direct_image_a6.exists() and direct_image_a6.is_dir():
        image_a6_folders.append(direct_image_a6)
    
    # 하위 폴더들에서 재귀적으로 찾기
    try:
        for subfolder in parent_folder.iterdir():
            if subfolder.is_dir() and subfolder.name != "image_a6":
                image_a6_folders.extend(find_image_a6_folders(subfolder))
    except PermissionError:
        print(f"Warning: Permission denied accessing {parent_folder}")
    
    return image_a6_folders


def get_image_files(folder: Path) -> List[Path]:
    """
    폴더에서 이미지 파일들을 찾습니다.
    
    Args:
        folder: 이미지 폴더
        
    Returns:
        이미지 파일 경로 리스트
    """
    image_extensions = {'.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif'}
    image_files = []
    
    try:
        for file_path in folder.iterdir():
            if file_path.is_file() and file_path.suffix.lower() in image_extensions:
                image_files.append(file_path)
    except PermissionError:
        print(f"Warning: Permission denied accessing {folder}")
    
    return sorted(image_files)


def resize_image(image_path: Path, target_size: Tuple[int, int] = (640, 384)) -> bool:
    """
    이미지를 지정된 크기로 리사이즈하여 덮어씁니다.
    
    Args:
        image_path: 리사이즈할 이미지 경로
        target_size: 목표 크기 (width, height)
        
    Returns:
        성공 여부
    """
    try:
        # 이미지 읽기
        image = cv2.imread(str(image_path))
        if image is None:
            print(f"Error: Could not read image {image_path}")
            return False
        
        # 현재 크기 확인
        current_height, current_width = image.shape[:2]
        target_width, target_height = target_size
        
        # 이미 목표 크기인 경우 건너뛰기
        if current_width == target_width and current_height == target_height:
            print(f"Skip: {image_path.name} is already {target_width}x{target_height}")
            return True
        
        # 리사이즈 수행
        resized_image = cv2.resize(image, target_size, interpolation=cv2.INTER_AREA)
        
        # 원본 파일에 덮어쓰기
        success = cv2.imwrite(str(image_path), resized_image)
        if success:
            print(f"Resized: {image_path.name} ({current_width}x{current_height} -> {target_width}x{target_height})")
            return True
        else:
            print(f"Error: Failed to save resized image {image_path}")
            return False
            
    except Exception as e:
        print(f"Error processing {image_path}: {e}")
        return False


def resize_images_in_folder(
    parent_folder: Path, 
    target_size: Tuple[int, int] = (640, 384),
    dry_run: bool = False
) -> None:
    """
    부모 폴더에서 image_a6 폴더들을 찾아 이미지들을 리사이즈합니다.
    
    Args:
        parent_folder: 검색할 부모 폴더
        target_size: 목표 크기 (width, height)
        dry_run: True이면 실제 리사이즈 없이 찾은 파일들만 출력
    """
    print(f"=== Image Resizer for image_a6 folders ===")
    print(f"Parent folder: {parent_folder}")
    print(f"Target size: {target_size[0]}x{target_size[1]}")
    if dry_run:
        print("DRY RUN MODE - No files will be modified")
    print()
    
    # parent_folder가 존재하는지 확인
    if not parent_folder.exists():
        print(f"Error: Parent folder does not exist: {parent_folder}")
        return
    
    if not parent_folder.is_dir():
        print(f"Error: Path is not a directory: {parent_folder}")
        return
    
    # image_a6 폴더들 찾기
    print("Searching for image_a6 folders...")
    image_a6_folders = find_image_a6_folders(parent_folder)
    
    if not image_a6_folders:
        print("No image_a6 folders found!")
        return
    
    print(f"Found {len(image_a6_folders)} image_a6 folder(s):")
    for folder in image_a6_folders:
        print(f"  - {folder}")
    print()
    
    # 각 image_a6 폴더에서 이미지 처리
    total_processed = 0
    total_success = 0
    total_failed = 0
    
    for folder in image_a6_folders:
        print(f"Processing folder: {folder}")
        
        # 이미지 파일들 찾기
        image_files = get_image_files(folder)
        
        if not image_files:
            print(f"  No image files found in {folder}")
            continue
        
        print(f"  Found {len(image_files)} image file(s)")
        
        if dry_run:
            for image_file in image_files:
                print(f"    Would resize: {image_file.name}")
            continue
        
        # 이미지들 리사이즈
        folder_success = 0
        folder_failed = 0
        
        for image_file in tqdm(image_files, desc=f"  Resizing images in {folder.name}"):
            total_processed += 1
            if resize_image(image_file, target_size):
                folder_success += 1
                total_success += 1
            else:
                folder_failed += 1
                total_failed += 1
        
        print(f"  Folder summary: {folder_success} success, {folder_failed} failed")
        print()
    
    # 전체 요약
    print(f"=== Summary ===")
    if dry_run:
        total_images = sum(len(get_image_files(folder)) for folder in image_a6_folders)
        print(f"Found {total_images} images across {len(image_a6_folders)} folders")
        print("Run without --dry-run to actually resize the images")
    else:
        print(f"Processed {len(image_a6_folders)} image_a6 folder(s)")
        print(f"Total images processed: {total_processed}")
        print(f"Successfully resized: {total_success}")
        print(f"Failed to resize: {total_failed}")


def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(
        description="Resize images in image_a6 folders to 640x384"
    )
    parser.add_argument(
        "--parent_folder", 
        type=str,
        help="Parent folder to search for image_a6 directories"
    )
    parser.add_argument(
        "--width", 
        type=int, 
        default=640,
        help="Target width (default: 640)"
    )
    parser.add_argument(
        "--height", 
        type=int, 
        default=384,
        help="Target height (default: 384)"
    )
    parser.add_argument(
        "--dry-run", 
        action="store_true",
        help="Show what would be done without actually modifying files"
    )
    
    args = parser.parse_args()
    
    parent_folder = Path(args.parent_folder)
    target_size = (args.width, args.height)
    
    resize_images_in_folder(parent_folder, target_size, args.dry_run)


if __name__ == "__main__":
    main()
