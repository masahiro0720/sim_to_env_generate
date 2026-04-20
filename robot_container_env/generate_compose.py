#!/usr/bin/env python3
"""
Docker Compose Generator for Robot Environments.
Fixed: Automated log directory creation with proper permissions to avoid Errno 13.
"""

import argparse
import os
import re
import shutil
import sys
from typing import List, Tuple, Dict, Any, Optional

# --- Constants ---
FASTDDS_XML_CONTENT = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS/fastRTPS_profiles.xsd">
    <participant profile_name="base_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
"""

ROS2_PATTERN = re.compile(r'FROM\s+.*ros:(humble|foxy|galactic|rolling|jazzy|iron)', re.IGNORECASE)
ROS1_PATTERN = re.compile(r'FROM\s+.*ros:(noetic|melodic|kinetic)', re.IGNORECASE)

def find_dockerfiles(dockerfile_dir: str):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    dockerfile_path_abs = os.path.join(script_dir, dockerfile_dir)
    if not os.path.isdir(dockerfile_path_abs): return [], None
    files = [f for f in os.listdir(dockerfile_path_abs) if f.endswith(".Dockerfile")]
    return sorted(files), dockerfile_path_abs

def select_dockerfiles(files: List[str]):
    print("📁 Found Dockerfiles:")
    for i, f in enumerate(files): print(f"  [{i+1}] {f}")
    choice = input("\n✅ Select numbers (e.g., '1,3' or 'all') > ").strip().lower()
    if not choice: return []
    if choice == "all": return files[:]
    try:
        return [files[int(x.strip()) - 1] for x in choice.split(',')]
    except: return []

def analyze_middleware(dockerfile_path_abs: str, filename: str) -> str:
    try:
        with open(os.path.join(dockerfile_path_abs, filename), 'r') as f:
            content = f.read()
        if ROS2_PATTERN.search(content): return "ROS2"
        if ROS1_PATTERN.search(content): return "ROS1"
        if "RTM" in filename.upper(): return "OpenRTM"
        return "Unknown"
    except: return "Error"

def get_next_compose_dir_path(base_dir: str):
    max_ver = 0
    if os.path.exists(base_dir):
        for entry in os.listdir(base_dir):
            if entry.startswith("compose_ver"):
                try: max_ver = max(max_ver, int(entry.replace("compose_ver", "")))
                except: pass
    return os.path.join(base_dir, f"compose_ver{max_ver + 1}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--flag', nargs='?', const='Integral')
    args, _ = parser.parse_known_args()
    integration_mode = (args.flag == 'Integral')

    files, dockerfile_path_abs = find_dockerfiles("./Dockerfile")
    if not files: return
    selected_files = select_dockerfiles(files)
    if not selected_files: return

    compose_base_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "compose_files")
    new_compose_dir = get_next_compose_dir_path(compose_base_dir)
    os.makedirs(new_compose_dir, exist_ok=True)

    # --- Compose YAML 組み立て ---
    lines = ["services:"]

    for i, dockerfile_name in enumerate(selected_files):
        service_name = f"module{i+1}"
        mw = analyze_middleware(dockerfile_path_abs, dockerfile_name)
        base_name = dockerfile_name.replace(".Dockerfile", "").replace("-", "_").lower()

        # ログパス設定
        log_path_in_container = ""
        if mw in ["ROS1", "ROS2"]:
            log_path_in_container = f"/home/rosuser/.ros/log"
        elif mw == "OpenRTM":
            log_path_in_container = f"/workspace/workspace/logs"

        # --- 重要: 改良点9の補完（権限エラー対策） ---
        # ホスト側にディレクトリをあらかじめ作り、権限を777にする
        if log_path_in_container:
            log_dir_on_host = os.path.join(new_compose_dir, "logs", service_name)
            os.makedirs(log_dir_on_host, exist_ok=True)
            os.chmod(log_dir_on_host, 0o777) # 誰でも書き込み可能にする
            # 親ディレクトリの権限も念のため確認
            os.chmod(os.path.join(new_compose_dir, "logs"), 0o777)

        # 手動でインデントを制御しながら組み立て
        lines.append(f"  {service_name}:")
        lines.append(f"    build:")
        lines.append(f"      context: .")
        lines.append(f"      dockerfile: {dockerfile_name}")
        lines.append(f"    image: {base_name}_image")
        lines.append(f"    container_name: {base_name}_container")

        lines.append(f"    # --- [Req 1] Enable Privileged Mode ---")
        lines.append(f"    privileged: true")

        lines.append(f"    # --- [Req 2] Use Host Network ---")
        lines.append(f"    network_mode: host")

        lines.append(f"    # --- [Req 3] Enable GUI Forwarding ---")
        lines.append(f"    environment:")
        lines.append(f"      - DISPLAY=${{DISPLAY}}")
        lines.append(f"      - QT_X11_NO_MITSHM=1")

        if integration_mode and mw == "ROS2":
            lines.append(f"      - FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_profile.xml")
            lines.append(f"    ipc: host")

        lines.append(f"    # --- OCI State Management: Volumes ---")
        lines.append(f"    volumes:")
        lines.append(f"      - /tmp/.X11-unix:/tmp/.X11-unix")
        lines.append(f"      - /dev:/dev")
        if log_path_in_container:
            lines.append(f"      - ./logs/{service_name}:{log_path_in_container}")
        if integration_mode and mw == "ROS2":
            lines.append(f"      - ./fastdds_profile.xml:/root/fastdds_profile.xml")

        lines.append(f"    # --- [Req 4] Interactive Terminal ---")
        lines.append(f"    tty: true")
        lines.append(f"    stdin_open: true")
        lines.append(f"    command: bash")

        lines.append(f"    # --- Auto-Restart Policy ---")
        lines.append(f"    restart: unless-stopped")

    # ファイル保存
    output_path = os.path.join(new_compose_dir, "docker-compose.yml")
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write("\n".join(lines))

    # Dockerfileコピー
    for filename in selected_files:
        shutil.copy(os.path.join(dockerfile_path_abs, filename), os.path.join(new_compose_dir, filename))

    # ROS2 Integration用XML作成
    if integration_mode:
        with open(os.path.join(new_compose_dir, "fastdds_profile.xml"), 'w') as f:
            f.write(FASTDDS_XML_CONTENT)

    print(f"\n🎉 Generated successfully: {new_compose_dir}")

if __name__ == "__main__":
    main()