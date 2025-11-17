import xml.etree.ElementTree as ET
import glob
import os
import re
import sys

def extract_profile_data(xml_file):
    """
    情報モデルXML (ISO22166-202準拠) から
    Dockerfile生成に必要なデータを抽出する。
    """
    try:
        tree = ET.parse(xml_file)
        root = tree.getroot()
    except FileNotFoundError:
        return {"error": f"Error: '{xml_file}' not found. Please check the file name."}
    except ET.ParseError as e:
        return {"error": f"Error: Failed to parse XML file. Please check the syntax.\nDetails: {e}"}

    # 1. APT/Git情報を格納している特定の<exeForm>を検索
    #    (Dockerfileのパスが含まれる<exeForm>を対象とする)
    exe_forms = root.findall('.//ExecutableForm/exeForm')
    target_exe_form = None

    for form in exe_forms:
        has_libraries = form.find("./additionalInfo/nv[Name='Libraries']/Value") is not None
        has_giturl = form.find("./additionalInfo/nv[Name='gitURL-name.1']/Value") is not None
        
        # Dockerfileパスの <exeFileURL> を検索
        exe_file_url_tag = form.find("./exeFileURL")
        has_dockerfile_url = (exe_file_url_tag is not None and
                              exe_file_url_tag.text is not None and
                              'Dockerfile' in exe_file_url_tag.text)

        if has_libraries and has_giturl and has_dockerfile_url:
            target_exe_form = form
            break

    if target_exe_form is None:
        return {"error": "Error: Target <exeForm> containing 'Libraries', 'gitURL-name.1', and 'Dockerfile' URL not found. Please check the XML structure."}

    # 2. ミドルウェア名とバージョンを <Infra> (または <Infrastructure>) タグから取得
    middleware_name = None
    middleware_version_min = None
    middleware_version_max = None

    infra_tag = root.find('.//Infra')
    if infra_tag is None:
        infra_tag = root.find('.//Infrastructure') # 旧形式もサポート

    if infra_tag is None:
        return {"error": "Error: <Infra> or <Infrastructure> tag not found. Cannot determine middleware."}

    # <middleware> タグをまず検索
    middleware_tag = infra_tag.find('./middleware')
    if middleware_tag is None:
         return {"error": "Error: <middleware> tag not found within <Infra>. Cannot determine middleware."}

    # 構造 1: <middleware><InfraType>...
    search_base = middleware_tag.find('./InfraType')
    
    if search_base is None:
        # 構造 2: <middleware>... (<InfraType> タグが省略されている場合)
        search_base = middleware_tag

    # <name> と <version> を抽出
    name_tag = search_base.find('name')
    version_tag = search_base.find('version')

    if name_tag is None:
        return {"error": "Error: <name> tag not found within middleware definition."}

    raw_middleware_name = name_tag.text.strip() if name_tag.text else ""

    # ミドルウェア名の正規化
    if raw_middleware_name == 'ROS 2':
        middleware_name = 'ROS 2'
    elif raw_middleware_name == 'ROS 1':
        middleware_name = 'ROS 1'
    elif raw_middleware_name == 'OpenRTM' or raw_middleware_name == 'RTM':
        middleware_name = 'OpenRTM'
    else:
        middleware_name = raw_middleware_name

    if middleware_name is None:
        return {"error": "Error: Middleware name not found in <InfraType>."}

    # ROS 1/2 の場合のみバージョンを <Infra> から取得
    if (middleware_name == 'ROS 1' or middleware_name == 'ROS 2'):
        if version_tag is None:
            return {"error": "Error: <version> tag not found for ROS in <InfraType>."}

        min_ver_raw = version_tag.attrib.get('min')
        max_ver_raw = version_tag.attrib.get('max')

        if not min_ver_raw or not max_ver_raw:
            return {"error": "Error: 'min' or 'max' attribute missing from <version> tag in <InfraType>."}

        middleware_version_min = min_ver_raw
        middleware_version_max = max_ver_raw

    # 3. OS名とバージョンを <Properties> から抽出
    os_info = root.find('.//osType') 
    if os_info is None:
        os_info = root.find('.//OSType') # 'OSType' (大文字) もサポート
    if os_info is None:
        return {"error": "Error: OS information not found. Please check the XML file."}

    os_name = os_info.attrib.get('type')

    os_version = None
    ver_range_os = root.find('.//compiler/verRangeOS')
    if ver_range_os is not None:
        os_version_min = ver_range_os.attrib.get('min')
        os_version_max = ver_range_os.attrib.get('max')
        os_version = f"min:{os_version_min},max:{os_version_max}"
    else:
        os_version = os_info.attrib.get('version')
        if os_version is not None:
            version_num = os_version.split(' ')[-1]
            os_version = f"min:{version_num},max:{version_num}"

    # コンパイラ情報を <Properties> から抽出
    compiler_name = None
    compiler_version = None
    compiler_info = root.find('.//compiler')
    if compiler_info is not None:
        compiler_name_element = compiler_info.find('compilerName')
        if compiler_name_element is not None:
            compiler_name = compiler_name_element.text

        ver_range_compiler = compiler_info.find('verRangeCompiler')
        if ver_range_compiler is not None:
            min_ver = ver_range_compiler.attrib.get('min', 'N/A')
            max_ver = ver_range_compiler.attrib.get('max', 'N/A')
            compiler_version = f"min:{min_ver},max:{max_ver}"

    # 4. APTパッケージのリストを抽出
    apt_packages = []
    library_nvs = target_exe_form.findall("./additionalInfo/nv[Name='Libraries']/Value")
    for value_tag in library_nvs:
        if value_tag.text:
            apt_packages.append(value_tag.text.strip())

    # 5. Gitリポジトリのリストを抽出 (gitURL-name.N 形式)
    git_repos = []
    repo_counter = 1

    while True:
        url_tag = target_exe_form.find(f"./additionalInfo/nv[Name='gitURL-name.{repo_counter}']/Value")
        branch_tag = target_exe_form.find(f"./additionalInfo/nv[Name='gitURL-branch.{repo_counter}']/Value")

        if url_tag is None:
            break

        repo_info = {}
        repo_info['id'] = f'gitURL{repo_counter}'
        repo_info['url'] = url_tag.text.strip()
        repo_info['branch'] = branch_tag.text.strip() if branch_tag is not None and branch_tag.text else ''

        git_repos.append(repo_info)
        repo_counter += 1

    return {
        "middleware_name": middleware_name,
        "middleware_version_min": middleware_version_min,
        "middleware_version_max": middleware_version_max,
        "os_name": os_name,
        "os_version": os_version,
        "apt_packages": apt_packages,
        "git_repos": git_repos,
        "compiler_name": compiler_name,
        "compiler_version": compiler_version
    }


def get_versions_by_range(min_ver, max_ver, is_ros_or_ubuntu):
    """
    ROSまたはUbuntuのバージョン辞書を使い、指定されたmin/max間のバージョンリストを返す。
    """
    if is_ros_or_ubuntu == 'ros':
        versions = {
            "kinetic": "16.04",
            "melodic": "18.04",
            "noetic": "20.04",
            "foxy": "20.04",
            "humble": "22.04",
            "jazzy": "24.04",
        }
    elif is_ros_or_ubuntu == 'ubuntu':
        versions = {
            "16.04": "xenial",
            "18.04": "bionic",
            "20.04": "focal",
            "22.04": "jammy",
            "24.04": "noble",
        }
    else:
        return []

    if min_ver not in versions or max_ver not in versions:
        if is_ros_or_ubuntu == 'ros' and (min_ver == 'noetic' or max_ver == 'jazzy'):
            pass
        else:
            return []

    keys = list(versions.keys())

    try:
        min_index = keys.index(min_ver)
        max_index = keys.index(max_ver)
    except ValueError:
        return []

    if min_index > max_index:
        return []

    available_versions = keys[min_index : max_index + 1]

    return available_versions

# メイン処理
if __name__ == "__main__":
    
    # --- 引数解析 ---
    if len(sys.argv) < 5: # config, version, txt_dir, xml_file1
        print("Error: Missing arguments.")
        print("Usage: python3 script.py [config_mode] [version_select_mode] [txt_output_dir] [xml_file1] [xml_file2] ...")
        sys.exit(1)

    config = sys.argv[1]
    version_select_mode = sys.argv[2]
    profile_output_dir = sys.argv[3] # .txt の出力先ディレクトリ
    xml_files_to_process = sys.argv[4:] # XMLファイルパスのリスト
    
    if not os.path.exists(profile_output_dir):
        print(f"Error: Output directory '{profile_output_dir}' does not exist.")
        sys.exit(1)

    if not xml_files_to_process:
        print("Error: No XML files specified for processing.")
    else:
        # --- メインループ ---
        for xml_file in xml_files_to_process:
            print(f"--- Starting processing of '{os.path.basename(xml_file)}' ---")
            
            profile_data = extract_profile_data(xml_file)

            if "error" in profile_data:
                print(profile_data["error"])
                continue

            middleware = profile_data['middleware_name']
            os_name = profile_data['os_name']

            # --- ミドルウェアがROSの場合の処理 ---
            if middleware == 'ROS 1' or middleware == 'ROS 2':
                min_ver = profile_data['middleware_version_min']
                max_ver = profile_data['middleware_version_max']

                if min_ver is None or max_ver is None:
                    print("Error: ROS version range (min/max) is not properly extracted from <Infra> tag.")
                    continue

                available_versions = get_versions_by_range(min_ver, max_ver, 'ros')

                if not available_versions:
                    print(f"Error: No matching ROS versions found for range '{min_ver}' to '{max_ver}'.")
                    print("--- End of processing ---")
                    continue

                # ROSバージョン選択
                selected_index = -1
                
                if version_select_mode == 'latest':
                    # 'latest' モード: 最新バージョンを自動選択
                    print(f"Auto-selecting latest version: {available_versions[-1]}")
                    selected_index = len(available_versions) - 1
                else:
                    # 'version_select' モード: 対話的に質問
                    print("Available ROS versions:")
                    for i, ver in enumerate(available_versions):
                        print(f"{i+1}: {ver}")
                        
                    while not (0 <= selected_index < len(available_versions)):
                        try:
                            user_input = input("Please select a version by number: ")
                            selected_index = int(user_input) - 1
                            if not (0 <= selected_index < len(available_versions)):
                                print("Invalid selection. Please choose a number from the list.")
                        except (ValueError, EOFError):
                            print("\nError: Input prompt received in non-interactive mode. Aborting.")
                            sys.exit(1)

                selected_version = available_versions[selected_index]

                # Dockerイメージ構成の選択
                if config:
                    if config == 'full':
                        ros_image = f"ros:{selected_version.lower()}-desktop-full"
                    else: # 'min'
                        ros_image = f"ros:{selected_version.lower()}-ros-base"
                else:
                    # config引数が無い場合
                    if version_select_mode == 'latest':
                        # 'latest'モードでは 'min' (ros-base) を自動選択
                        print("Auto-selecting minimal configuration (ros-base).")
                        ros_image = f"ros:{selected_version.lower()}-ros-base"
                    else:
                        # 'version_select' モードでは追加で質問
                        print("\nPlease select Docker image configuration:")
                        print("1: Minimal configuration (ros-base)")
                        print("2: Full package configuration (desktop-full)")

                        config_index = -1
                        while not (0 <= config_index < 2):
                            try:
                                user_input = input("Enter the number for your choice: ")
                                config_index = int(user_input) - 1
                                if not (0 <= config_index < 2):
                                    print("Invalid selection. Please enter 1 or 2.")
                                else:
                                    if config_index == 1:
                                        ros_image = f"ros:{selected_version.lower()}-desktop-full"
                                    else:
                                        ros_image = f"ros:{selected_version.lower()}-ros-base"
                            except (ValueError, EOFError):
                                print("\nError: Input prompt received in non-interactive mode. Aborting.")
                                sys.exit(1)

                ros_distro = selected_version.lower()
                output_file_config = 'desktop-full' if 'desktop-full' in ros_image else 'ros-base'

                # APTパッケージの${ROS_DISTRO}置換
                updated_apt_packages = []
                for pkg in profile_data['apt_packages']:
                    updated_apt_packages.append(pkg.replace('${ROS_DISTRO}', ros_distro))

                # Gitブランチ名の処理 ('required' を置換)
                updated_git_repos = []
                for repo in profile_data['git_repos']:
                    repo_branch = repo['branch']
                    if repo_branch == 'required':
                        updated_git_repos.append({**repo, 'branch': ros_distro})
                    else:
                        updated_git_repos.append(repo)

                # プロファイルファイル書き出し
                base_name = os.path.splitext(os.path.basename(xml_file))[0]
                output_filename = f"{base_name}_ROS_{selected_version}_{output_file_config}_profile.txt"
                output_file = os.path.join(profile_output_dir, output_filename)

                with open(output_file, 'w') as f:
                    f.write(f"ROS_IMAGE: {ros_image}\n")
                    f.write(f"ROS_DISTRO: {ros_distro}\n")
                    f.write("\nAPT_PACKAGES:\n")
                    for pkg in updated_apt_packages:
                        f.write(f"- {pkg}\n")
                    f.write("\nGIT_REPOSITORIES:\n")
                    for repo in updated_git_repos:
                        f.write(f"- {repo['id']}\n")
                        f.write(f"  url: {repo['url']}\n")
                        f.write(f"  branch: {repo['branch'] if repo['branch'] else '(none)'}\n")

                print(f"Extracted data was written to '{output_file}'.")

            # --- ミドルウェアがOpenRTMの場合の処理 ---
            elif middleware == 'OpenRTM':
                os_version_range = profile_data['os_version']
                match = re.search(r"min:(.+),max:(.+)", os_version_range)
                if not match:
                    print(f"Error: Invalid OS version range format '{os_version_range}'.")
                    print("--- End of processing ---")
                    continue

                min_ver = match.group(1).strip()
                max_ver = match.group(2).strip()

                available_versions = get_versions_by_range(min_ver, max_ver, 'ubuntu')

                if not available_versions:
                    print(f"Error: No matching Ubuntu versions found for range '{min_ver}' to '{max_ver}'.")
                    print("--- End of processing ---")
                    continue

                # Ubuntuバージョン選択
                selected_index = -1
                
                if version_select_mode == 'latest':
                    # 'latest' モード: 最新バージョンを自動選択
                    print(f"Auto-selecting latest version: {available_versions[-1]}")
                    selected_index = len(available_versions) - 1
                else:
                    # 'version_select' モード: 対話的に質問
                    print("Available Ubuntu versions:")
                    for i, ver in enumerate(available_versions):
                        print(f"{i+1}: {ver}")

                    while not (0 <= selected_index < len(available_versions)):
                        try:
                            user_input = input("Please select a version by number: ")
                            selected_index = int(user_input) - 1
                            if not (0 <= selected_index < len(available_versions)):
                                print("Invalid selection. Please choose a number from the list.")
                        except (ValueError, EOFError):
                            print("\nError: Input prompt received in non-interactive mode. Aborting.")
                            sys.exit(1)

                selected_os_version = available_versions[selected_index]

                # プロファイルファイル書き出し
                base_name = os.path.splitext(os.path.basename(xml_file))[0]
                output_filename = f"{base_name}_RTM_{selected_os_version}_profile.txt"
                output_file = os.path.join(profile_output_dir, output_filename)

                with open(output_file, 'w') as f:
                    f.write(f"MIDDLEWARE_NAME: {profile_data['middleware_name']}\n")
                    f.write(f"OS_NAME: {os_name}\n")
                    f.write(f"OS_VERSION: {selected_os_version}\n")
                    if 'compiler_name' in profile_data:
                        f.write(f"COMPILER_NAME: {profile_data['compiler_name']}\n")
                    if 'compiler_version' in profile_data:
                        f.write(f"COMPILER_VERSION: {profile_data['compiler_version']}\n")

                    f.write("\nAPT_PACKAGES:\n")
                    for pkg in profile_data['apt_packages']:
                        f.write(f"- {pkg}\n")
                    f.write("\nGIT_REPOSITORIES:\n") 
                    for repo in profile_data['git_repos']:
                        f.write(f"- {repo['id']}\n")
                        f.write(f"  url: {repo['url']}\n")
                        if repo['branch']:
                            f.write(f"  branch: {repo['branch']}\n")
                        else:
                            f.write(f"  branch: (none)\n")
                print(f"Extracted data was written to '{output_file}'.")

            else:
                print(f"Error: Unknown middleware '{middleware}' detected.")
            print("--- End of processing ---")
