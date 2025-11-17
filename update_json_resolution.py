#!/usr/bin/env python3
"""
JSON 파일에서 640x384를 640x512로 일괄 변경
"""

import json
import sys
from pathlib import Path

def update_json_resolution(json_path: Path, old_res: str = "640x384", new_res: str = "640x512"):
    """JSON 파일에서 해상도 문자열 변경"""
    print(f"\n[Processing] {json_path.name}")
    
    # JSON 로드
    with open(json_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # 640x384 → 640x512 변경
    original_count = content.count(old_res)
    updated_content = content.replace(old_res, new_res)
    
    if original_count > 0:
        # 백업 저장
        backup_path = json_path.with_suffix('.json.bak')
        with open(backup_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"  ✅ Backup saved: {backup_path.name}")
        
        # 업데이트된 내용 저장
        with open(json_path, 'w', encoding='utf-8') as f:
            f.write(updated_content)
        
        print(f"  ✅ Updated {original_count} occurrences: {old_res} → {new_res}")
        print(f"  ✅ Saved: {json_path}")
    else:
        print(f"  ℹ️ No '{old_res}' found in file")
    
    return original_count

def main():
    """메인 함수"""
    if len(sys.argv) < 2:
        print("Usage: python update_json_resolution.py <json_file_or_directory>")
        print("\nExamples:")
        print("  python update_json_resolution.py combined_train.json")
        print("  python update_json_resolution.py D:\\data\\ncdb-cls\\ncdb-cls-640x512\\splits")
        return
    
    input_path = Path(sys.argv[1])
    
    if not input_path.exists():
        print(f"❌ Error: Path does not exist: {input_path}")
        return
    
    # 디렉토리인 경우 모든 JSON 파일 처리
    if input_path.is_dir():
        json_files = list(input_path.glob("*.json"))
        print(f"\n{'='*80}")
        print(f"Found {len(json_files)} JSON files in: {input_path}")
        print(f"{'='*80}")
        
        total_changes = 0
        for json_file in json_files:
            count = update_json_resolution(json_file)
            total_changes += count
        
        print(f"\n{'='*80}")
        print(f"✅ Complete! Total {total_changes} occurrences updated across {len(json_files)} files")
        print(f"{'='*80}\n")
    
    # 파일인 경우 단일 파일 처리
    elif input_path.is_file() and input_path.suffix == '.json':
        print(f"\n{'='*80}")
        print(f"Processing single file: {input_path}")
        print(f"{'='*80}")
        
        count = update_json_resolution(input_path)
        
        print(f"\n{'='*80}")
        print(f"✅ Complete! {count} occurrences updated")
        print(f"{'='*80}\n")
    
    else:
        print(f"❌ Error: Invalid input. Please provide a JSON file or directory.")

if __name__ == "__main__":
    main()
