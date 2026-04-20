import xml.etree.ElementTree as ET
import glob
import os
import re
import sys
import json

# 写像データベースのパス (外部ファイルを読み込む)
DB_FILE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'mapping_db.json')

def load_mapping_db():
    """写像データベース(JSON)を読み込む"""
    if not os.path.exists(DB_FILE_PATH):
        print(f"Warning: Mapping DB not found at {DB_FILE_PATH}. Using empty DB.")
        return {}
    try:
        with open(DB_FILE_PATH, 'r') as f:
            return json.load(f)
    except json.JSONDecodeError as e:
        print(f"Error: Failed to parse mapping DB JSON: {e}")
        return {}

def resolve_package_name(logical_name, db, context):
    """
    論理ライブラリ名を物理パッケージ名に変換する
    """
    # 1. DB検索
    if 'libraries' in db:
        lib_entry = db['libraries'].get(logical_name)
        if lib_entry:
            physical_target = None
            pkg_type = 'apt' # Default

            mw = context.get('middleware')
            os_type = context.get('os')
            lang = context.get('lang')

            # Middleware Check
            if mw and mw in lib_entry:
                if 'apt' in lib_entry[mw]:
                    physical_target = lib_entry[mw]['apt']
                    pkg_type = 'apt'
                elif 'pip' in lib_entry[mw]:
                    physical_target = lib_entry[mw]['pip']
                    pkg_type = 'pip'

            # Language Check
            if not physical_target and lang == 'python' and 'python' in lib_entry:
                if 'pip' in lib_entry['python']:
                    physical_target = lib_entry['python']['pip']
                    pkg_type = 'pip'
                elif 'apt' in lib_entry['python']:
                     physical_target = lib_entry['python']['apt']
                     pkg_type = 'apt'

            # OS Check
            if not physical_target and os_type and os_type in lib_entry:
                if 'apt' in lib_entry[os_type]:
                    physical_target = lib_entry[os_type]['apt']
                    pkg_type = 'apt'
                elif 'pip' in lib_entry[os_type]:
                    physical_target = lib_entry[os_type]['pip']
                    pkg_type = 'pip'

            # Default Check
            if not physical_target and 'default' in lib_entry:
                if 'apt' in lib_entry['default']:
                    physical_target = lib_entry['default']['apt']
                    pkg_type = 'apt'
                elif 'pip' in lib_entry['default']:
                    physical_target = lib_entry['default']['pip']
                    pkg_type = 'pip'

            if physical_target:
                if '${ROS_DISTRO}' in physical_target and 'ros_distro' in context:
                     physical_target = physical_target.replace('${ROS_DISTRO}', context['ros_distro'])
                return {'type': pkg_type, 'packages': physical_target.split()}

    # 2. DBにない場合のフォールバックロジック
    if context.get('lang') == 'python':
        return {'type': 'pip', 'packages': [logical_name]}

    return {'type': 'apt', 'packages': [logical_name]}

def extract_profile_data(xml_file):
    try:
        tree = ET.parse(xml_file)
        root = tree.getroot()
    except Exception as e:
        return {"error": f"Error parsing XML: {e}"}

    # --- 1. GenInfoからのメタデータ取得 ---
    module_name = "UnknownModule"
    description = ""
    authors = ""

    gen_info = root.find('.//GenInfo')
    if gen_info is not None:
        name_tag = gen_info.find('ModuleName')
        if name_tag is not None and name_tag.text:
            module_name = name_tag.text.strip()

        desc_tag = gen_info.find('Description')
        if desc_tag is not None and desc_tag.text:
            description = desc_tag.text.strip()

        mf_tag = gen_info.find('Manufacturer')
        if mf_tag is not None:
            # Authors タグの取得を試みる
            auth_tag = mf_tag.find('Authors')
            if auth_tag is not None and auth_tag.text:
                authors = auth_tag.text.strip()
            else:
                # Authorsがない場合は Name (組織名等) でフォールバックする
                name_in_mf_tag = mf_tag.find('Name')
                if name_in_mf_tag is not None and name_in_mf_tag.text:
                    authors = name_in_mf_tag.text.strip()

    # --- 2. ExecutableForm / exeForm (dockerfile_) からのデータ抽出 ---
    exe_forms = root.findall('.//ExecutableForm/exeForm')
    target_exeform = None
    for ef in exe_forms:
        file_url = getattr(ef.find('exeFileURL'), 'text', '')
        if file_url and file_url.startswith('dockerfile_'):
            target_exeform = ef
            break

    if target_exeform is None:
        return {"error": f"Error: No <exeForm> with 'dockerfile_...' found in {xml_file}."}

    # 初期値
    middleware_name = ""
    middleware_version_min = ""
    middleware_version_max = ""
    os_name = "Ubuntu"
    os_version = "20.04"
    workspace = ""
    compiler_name = "GCC" # Default
    logical_libs = []
    git_repos = []

    # additionalInfo 内の <nv> タグをすべて解析
    for nv in target_exeform.findall('.//additionalInfo/nv'):
        name = getattr(nv.find('name'), 'text', '')
        value = getattr(nv.find('value'), 'text', '')
        
        if name == 'dockerfile_Middleware': 
            middleware_name = value
        elif name == 'dockerfile_MiddlewareVersion': 
            middleware_version_min = value.lower()
            middleware_version_max = value.lower()
        elif name == 'dockerfile_TargetOSVersion': 
            # "Ubuntu 20.04 (Focal)" から "20.04" を抽出
            m = re.search(r'\d+\.\d+', value)
            if m: os_version = m.group(0)
        elif name == 'dockerfile_Workspace': 
            workspace = value
        elif name == 'dockerfile_Language': 
            compiler_name = value.replace('dockerfile_', '')
        elif name == 'dockerfile_Libraries': 
            logical_libs.append(value)
        elif name == 'dockerfile_GitURL':
            parts = value.split(' ')
            url = parts[0]
            branch = parts[1] if len(parts) > 1 else 'main'
            git_repos.append({'id': f'gitURL{len(git_repos)+1}', 'url': url, 'branch': branch})

    if not middleware_name:
        return {"error": "Error: Middleware information not found in exeForm."}

    # --- 3. パッケージの推論と変換ロジック (既存コードを活かす) ---
    db = load_mapping_db()
    apt_packages = set()
    pip_packages = set()

    lang_context = 'python' if 'python' in compiler_name.lower() else 'c++'
    mw_context = 'ros1' if '1' in middleware_name else ('ros2' if '2' in middleware_name else 'openrtm')

    context = {
        'os': os_name.lower(),
        'os_version': os_version,
        'middleware': mw_context,
        'lang': lang_context,
        'ros_distro': middleware_version_min # 置換用
    }

    for logical_name in logical_libs:
        resolved = resolve_package_name(logical_name, db, context)
        if resolved:
            if resolved['type'] == 'apt':
                for p in resolved['packages']: apt_packages.add(p)
            elif resolved['type'] == 'pip':
                for p in resolved['packages']: pip_packages.add(p)

    # [Rule 1] 基本開発ツール (Always)
    base_tools = ['git', 'build-essential', 'cmake', 'python3-pip', 'libboost-all-dev']
    for tool in base_tools:
        res = resolve_package_name(tool, db, context)
        if res and res['type'] == 'apt': apt_packages.update(res['packages'])

    # [Rule 2] ROS 1 Toolset
    if mw_context == 'ros1':
        ros1_tools = [
            'python3-catkin-tools', 'python3-rosdep', 'python3-rosinstall',
            'python3-rosinstall-generator', 'python3-wstool'
        ]
        if lang_context == 'python': ros1_tools.append('rospy')
        for tool in ros1_tools:
            res = resolve_package_name(tool, db, context)
            if res and res['type'] == 'apt': apt_packages.update(res['packages'])

    # [Rule 3] ROS 2 Toolset
    if mw_context == 'ros2':
        ros2_tools = ['python3-colcon-common-extensions', 'python3-rosdep']
        if lang_context == 'python': ros2_tools.extend(['rclpy', 'python3-setuptools'])
        for tool in ros2_tools:
            res = resolve_package_name(tool, db, context)
            if res and res['type'] == 'apt': apt_packages.update(res['packages'])

    # [Rule 4] OpenRTM Toolset (開発用依存関係の追加)
    if mw_context == 'openrtm':
        rtm_tools = ['python3-dev', 'omniidl', 'omniorb-nameserver', 'doxygen', 'openrtm-aist-dev', 'pkg-config', 'uuid-dev']
        is_old_ubuntu = (context.get('os_version') in ['18.04', '16.04'])
        if not is_old_ubuntu: rtm_tools.append('python-is-python3')
        for tool in rtm_tools:
            res = resolve_package_name(tool, db, context)
            if res and res['type'] == 'apt': apt_packages.update(res['packages'])

    # [Rule 5] GUI/Sim関連
    gui_triggers = ['rviz', 'rviz2', 'gazebo_ros', 'rqt_gui', 'opencv2', 'vision_opencv']
    needs_gui = any(lib in gui_triggers for lib in logical_libs)
    if needs_gui:
        gui_libs = ['x11-apps', 'mesa-utils', 'libgl1-mesa-glx', 'libgl1-mesa-dri']
        for lib in gui_libs:
            res = resolve_package_name(lib, db, context)
            if res and res['type'] == 'apt': apt_packages.update(res['packages'])

    # [Rule 6] ハードウェア制御関連
    hw_triggers = ['dynamixel_sdk', 'DynamixelSDK', 'pyserial', 'ros_control', 'ros2_control', 'usb_cam']
    needs_hw = any(lib in hw_triggers for lib in logical_libs)
    if needs_hw:
        res = resolve_package_name('libudev-dev', db, context)
        if res and res['type'] == 'apt': apt_packages.update(res['packages'])

    return {
        "module_name": module_name,
        "description": description,
        "authors": authors,
        "middleware_name": middleware_name,
        "middleware_version_min": middleware_version_min,
        "middleware_version_max": middleware_version_max,
        "os_name": os_name,
        "os_version": os_version,
        "workspace": workspace, # 追加: exeFormから取得したワークスペース名
        "apt_packages": sorted(list(apt_packages)),
        "pip_packages": sorted(list(pip_packages)),
        "git_repos": git_repos,
        "compiler_name": compiler_name,
        "compiler_version": "N/A"
    }

def get_versions_by_range(min_ver, max_ver, is_ros_or_ubuntu):
    if is_ros_or_ubuntu == 'ros':
        versions = {"kinetic": "16.04", "melodic": "18.04", "noetic": "20.04", "foxy": "20.04", "humble": "22.04", "jazzy": "24.04"}
    elif is_ros_or_ubuntu == 'ubuntu':
        versions = {"16.04": "xenial", "18.04": "bionic", "20.04": "focal", "22.04": "jammy", "24.04": "noble"}
    else: return []

    keys = list(versions.keys())
    try: min_idx = keys.index(min_ver)
    except: min_idx = 0
    try: max_idx = keys.index(max_ver)
    except: max_idx = len(keys) - 1

    if min_idx > max_idx: return []
    return keys[min_idx : max_idx + 1]

if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("Error: Missing arguments.")
        sys.exit(1)

    config = sys.argv[1]
    version_select_mode = sys.argv[2]
    profile_output_dir = sys.argv[3]
    xml_files_to_process = sys.argv[4:]

    if not os.path.exists(profile_output_dir):
        print(f"Error: Output directory not found.")
        sys.exit(1)

    for xml_file in xml_files_to_process:
        print(f"--- Processing '{os.path.basename(xml_file)}' ---")

        profile_data = extract_profile_data(xml_file)
        if "error" in profile_data:
            print(profile_data["error"])
            continue

        mw = profile_data['middleware_name']
        mod_name = profile_data['module_name']

        if mw.startswith('ROS'):
            min_ver = profile_data['middleware_version_min']
            max_ver = profile_data['middleware_version_max']
            avail_vers = get_versions_by_range(min_ver, max_ver, 'ros')

            if not avail_vers:
                print("Error: No matching ROS versions found.")
                continue

            selected_ver = None
            if version_select_mode == 'latest':
                selected_ver = avail_vers[-1]
                print(f"Auto-selecting latest version: {selected_ver}")
            elif version_select_mode in ['select', 'version_select']:
                print("Available ROS versions:")
                for i, ver in enumerate(avail_vers):
                    print(f"{i+1}: {ver}")
                while True:
                    try:
                        sel = int(input("Please select a version by number: ")) - 1
                        if 0 <= sel < len(avail_vers):
                            selected_ver = avail_vers[sel]
                            break
                    except: pass
                    print("Invalid selection.")
            else:
                 selected_ver = avail_vers[-1]

            if config == 'full':
                # Foxyにはdesktop-fullが存在しないため、desktopにフォールバックする
                if selected_ver == 'foxy':
                    ros_image_tag = "desktop"
                else:
                    ros_image_tag = "desktop-full"
                config_label = "Full"
                ros_repo = "docker.io/osrf/ros"
            else:
                ros_image_tag = "ros-base"
                config_label = "Min"
                ros_repo = "docker.io/library/ros"

            ros_image = f"{ros_repo}:{selected_ver}-{ros_image_tag}"

            versions_map = {"kinetic": "16.04", "melodic": "18.04", "noetic": "20.04", "foxy": "20.04", "humble": "22.04", "jazzy": "24.04"}
            os_ver = versions_map.get(selected_ver, "Unknown")

            mw_label = "ROS1" if "1" in mw or selected_ver in ["kinetic", "melodic", "noetic"] else "ROS2"

            out_name = f"{mod_name}__Ubuntu-{os_ver}__{mw_label}-{selected_ver}__{config_label}_profile.txt"
            out_path = os.path.join(profile_output_dir, out_name)

            final_apt = [p.replace('${ROS_DISTRO}', selected_ver) for p in profile_data['apt_packages']]
            final_git = []
            for r in profile_data['git_repos']:
                br = r['branch']
                if br == 'required': br = selected_ver
                final_git.append({**r, 'branch': br})

            with open(out_path, 'w') as f:
                f.write(f"ROS_IMAGE: {ros_image}\n")
                f.write(f"ROS_DISTRO: {selected_ver}\n")
                f.write(f"COMPILER_NAME: {profile_data['compiler_name']}\n")
                f.write(f"WORKSPACE_NAME: {profile_data['workspace']}\n")
                f.write(f"DESCRIPTION: {profile_data['description']}\n")
                f.write(f"AUTHORS: {profile_data['authors']}\n")
                f.write("\nAPT_PACKAGES:\n")
                for p in final_apt: f.write(f"- {p}\n")
                f.write("\nPIP_PACKAGES:\n")
                for p in profile_data['pip_packages']: f.write(f"- {p}\n")
                f.write("\nGIT_REPOSITORIES:\n")
                for r in final_git:
                    f.write(f"- {r['id']}\n  url: {r['url']}\n  branch: {r['branch'] if r['branch'] else '(none)'}\n")
            print(f"Generated: {out_path}")

        elif mw == 'OpenRTM':
            # RTMはexeFormに記述されたOSバージョンをそのまま使用
            os_ver = profile_data['os_version']
            mw_ver = profile_data['middleware_version_min'] or "1.0.0"

            out_name = f"{mod_name}__Ubuntu-{os_ver}__OpenRTM-{mw_ver}__Std_profile.txt"
            out_path = os.path.join(profile_output_dir, out_name)

            with open(out_path, 'w') as f:
                f.write(f"MIDDLEWARE_NAME: OpenRTM\n")
                f.write(f"OS_NAME: {profile_data['os_name']}\n")
                f.write(f"OS_VERSION: {os_ver}\n")
                f.write(f"COMPILER_NAME: {profile_data['compiler_name']}\n")
                f.write(f"WORKSPACE_NAME: {profile_data['workspace']}\n")
                f.write(f"DESCRIPTION: {profile_data['description']}\n")
                f.write(f"AUTHORS: {profile_data['authors']}\n")
                f.write("\nAPT_PACKAGES:\n")
                for p in profile_data['apt_packages']: f.write(f"- {p}\n")
                f.write("\nPIP_PACKAGES:\n")
                for p in profile_data['pip_packages']: f.write(f"- {p}\n")
                f.write("\nGIT_REPOSITORIES:\n")
                for r in profile_data['git_repos']:
                    br = 'main' if r['branch'] == 'required' else r['branch']
                    f.write(f"- {r['id']}\n  url: {r['url']}\n  branch: {br if br else '(none)'}\n")
            print(f"Generated: {out_path}")

        print("--- Done ---")