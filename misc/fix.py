import os
import re

# 폴더 매핑 정의 (기존 모듈 이름 -> 새로운 폴더 경로)
module_to_folder = {
    'camera': 'vision.camera',
    'board': 'vision.board',
    'apriltag': 'vision.apriltag',
    'tracking': 'vision.tracking',
    'cbs_runner': 'cbs.cbs_runner',
    'cbs_manager': 'cbs.cbs_manager',
    'path_relay': 'cbs.path_relay',
    'movement_generator': 'movement.movement_generator'
}

# 수정할 루트 디렉터리 (여기에 main.py, vision/, cbs/ 등이 있는 곳)
root_dir = 'D:\git\Capstone_Design-OpenCV\code'  # <-- 여기에 본인 경로 입력 (예: "D:/git/Capstone_temp")

def fix_imports_in_file(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    changed = False
    new_lines = []
    for line in lines:
        for module, new_path in module_to_folder.items():
            # import 모듈
            if re.match(rf'^\s*import\s+{module}\b', line):
                line = re.sub(rf'\b{module}\b', new_path, line)
                changed = True
            # from 모듈 import XXX
            elif re.match(rf'^\s*from\s+{module}\b', line):
                line = re.sub(rf'\b{module}\b', new_path, line)
                changed = True
        new_lines.append(line)

    if changed:
        with open(file_path, 'w', encoding='utf-8') as f:
            f.writelines(new_lines)
        print(f"✅ 수정 완료: {file_path}")

def walk_and_fix_imports(root_dir):
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith('.py'):
                fix_imports_in_file(os.path.join(root, file))

if __name__ == '__main__':
    walk_and_fix_imports(root_dir)
