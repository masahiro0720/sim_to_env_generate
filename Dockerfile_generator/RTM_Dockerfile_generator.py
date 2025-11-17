import re
import glob
import os
import sys

def parse_profile_data(file_path):
    """
    プロファイルファイル (.txt) を読み込み、Dockerfile生成に必要な情報を抽出する。
    """
    data = {}
    try:
        with open(file_path, 'r') as f:
            content = f.read()

        # ミドルウェア情報を抽出
        middleware_name_match = re.search(r"MIDDLEWARE_NAME: (.+)", content)
        if middleware_name_match:
            data['middleware_name'] = middleware_name_match.group(1).strip()
        middleware_version_match = re.search(r"MIDDLEWARE_VERSION: (.+)", content)
        if middleware_version_match:
            data['middleware_version'] = middleware_version_match.group(1).strip()

        # OS情報を抽出
        os_name_match = re.search(r"OS_NAME: (.+)", content)
        if os_name_match:
            data['os_name'] = os_name_match.group(1).strip()
        os_version_match = re.search(r"OS_VERSION: (.+)", content)
        if os_version_match:
            data['os_version'] = os_version_match.group(1).strip()

        # APTパッケージを抽出
        apt_packages_match = re.search(r"APT_PACKAGES:(.+?)GIT_REPOSITORIES:", content, re.DOTALL)
        if apt_packages_match:
            apt_packages = re.findall(r"- (.+)", apt_packages_match.group(1))
            data['apt_packages'] = [pkg.strip() for pkg in apt_packages]
        else:
            data['apt_packages'] = []

        # Gitリポジトリを抽出 (連番ID形式に対応)
        git_repos_match = re.search(r"GIT_REPOSITORIES:(.+)", content, re.DOTALL)
        git_repos = []
        if git_repos_match:
            repo_block = git_repos_match.group(1).strip()
            # '- gitURLN' を区切りとして、IDと内容をペアで抽出
            repo_sections = re.split(r"-\s*(gitURL\d+)", repo_block)

            # i=1から開始し、IDと内容を順に処理
            for i in range(1, len(repo_sections), 2):
                repo_id = repo_sections[i].strip()
                section_content = repo_sections[i+1].strip()

                repo = {}
                repo['name'] = repo_id # 構造維持のためIDを格納

                url_match = re.search(r"url: (.+)", section_content)
                branch_match = re.search(r"branch: (.+)", section_content)

                repo['url'] = url_match.group(1).strip() if url_match else None
                repo['branch'] = branch_match.group(1).strip() if branch_match and branch_match.group(1).strip() != '(none)' else None

                if repo['url']: # URLがある場合のみ追加
                    git_repos.append(repo)

        data['git_repos'] = git_repos

        # コンパイラ情報を抽出
        compiler_name_match = re.search(r"COMPILER_NAME: (.+)", content)
        if compiler_name_match:
            data['compiler_name'] = compiler_name_match.group(1).strip()
        compiler_version_match = re.search(r"COMPILER_VERSION: (.+)", content)
        if compiler_version_match:
            data['compiler_version'] = compiler_version_match.group(1).strip()

        return data

    except FileNotFoundError:
        print(f"Error: '{file_path}' not found. Please check the file name.")
        return None
    except Exception as e:
        print(f"Error: An error occurred during parsing: {e}")
        return None

def generate_dockerfile(data):
    """
    抽出したデータからDockerfileを生成する
    """

    # OSバージョン名 (e.g., '20.04' or 'focal') を取得
    os_version = data['os_version'].strip()
    os_name_lower = data['os_name'].lower()

    if os_name_lower == 'linux':
        os_name_lower = 'ubuntu'

    # Ubuntuのバージョンに対応するコードネームを取得
    os_codename_map = {
        "20.04": "focal",
        "18.04": "bionic",
        "16.04": "xenial",
    }
    # os_version が 'focal' 等でも '20.04' 等でも対応
    os_codename = os_codename_map.get(os_version, os_version)
    
    # DockerfileのFROM行に使用するイメージ名
    base_image = f"{os_name_lower}:{os_codename}"

    # OpenRTMのAPTリポジトリURLを動的に生成
    openrtm_repo_url = f"deb [trusted=yes] http://openrtm.org/pub/Linux/ubuntu {os_codename} main"

    # APTパッケージのインストールコマンドを整形
    apt_packages_str = " \\\n    ".join(data['apt_packages'])

    # Gitクローンとビルドのコマンドを生成
    build_commands = ""
    for repo in data['git_repos']:
        if repo['url']:
            repo_name = repo['url'].split('/')[-1].replace('.git', '')

            if repo['branch'] and repo['branch'] != 'main':
                branch_command = f"-b {repo['branch']}"
            else:
                branch_command = ""

            # コンパイラが 'GCC' の場合のみ cmake/make を実行
            if data.get('compiler_name') == 'GCC':
                build_commands += f"""
# Clone and build repository: {repo['name']}
RUN git clone {branch_command} {repo['url']} \\
    && cd {repo_name} \\
    && git submodule update --init --recursive \\
    && mkdir -p build && cd build \\
    && cmake .. -DBUILD_DOCUMENTATION=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \\
    && make -j$(nproc)
"""
            else:
                # GCC以外の場合はクローンのみ
                build_commands += f"""
# Clone repository: {repo['name']} (Build commands skipped for non-GCC compiler)
RUN git clone {branch_command} {repo['url']} \\
    && cd {repo_name}
"""

    # Dockerfile全体のコンテンツを構築
    dockerfile_content = f"""# Base image: {data['os_name']} {os_version}
FROM {base_image}

# Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set working directory
WORKDIR /workspace

# Add OpenRTM-aist repository and install packages
RUN rm -f /etc/apt/sources.list.d/openrtm.list \\
    && echo "{openrtm_repo_url}" > /etc/apt/sources.list.d/openrtm.list \\
    && apt-get update \\
    && apt-get install -y \\
    {apt_packages_str} \\
    && apt-get clean \\
    && rm -rf /var/lib/apt/lists/*
{build_commands}

# Enter the container with a login shell
CMD ["bash", "-l"]
"""
    return dockerfile_content


if __name__ == "__main__":

    # --- 引数解析 ---
    if len(sys.argv) < 3:
        print("Error: Missing arguments.")
        print("Usage: python3 script.py [profile_file_path] [output_dir_path]")
        sys.exit(1)
        
    profile_file_to_process = sys.argv[1] # 親から渡された .txt ファイルのフルパス
    output_dir = sys.argv[2]              # 親から渡された Dockerfile 出力先のフルパス

    if not os.path.exists(profile_file_to_process):
        print(f"Error: Profile file '{profile_file_to_process}' not found.")
    else:
        print("-" * 40)
        print(f"--- Starting processing of '{os.path.basename(profile_file_to_process)}' ---")

        profile_data = parse_profile_data(profile_file_to_process)

        if profile_data:
            dockerfile_content = generate_dockerfile(profile_data)

            # 出力パスを引数で指定されたディレクトリに変更
            file_name_without_ext = os.path.splitext(os.path.basename(profile_file_to_process))[0]
            output_dockerfile = os.path.join(output_dir, f"{file_name_without_ext}.Dockerfile")

            with open(output_dockerfile, 'w') as f:
                f.write(dockerfile_content)

            print(f"Dockerfile generated successfully and written to '{output_dockerfile}'.")

            print(f"\n--- Content of '{output_dockerfile}' ---")
            print(dockerfile_content)

        print("-" * 40)
