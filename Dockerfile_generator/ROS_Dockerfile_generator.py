import re
import glob
import os
import sys

def parse_profile_data(file_path):
    """
    プロファイルファイル（.txt）を解析し、Dockerfile生成に必要なデータを抽出する。
    """
    data = {}
    try:
        with open(file_path, 'r') as f:
            content = f.read()

        # [ROS_IMAGE]の抽出とROSバージョンの判定
        ros_image_match = re.search(r"ROS_IMAGE: (.+)", content)
        if ros_image_match:
            ros_image = ros_image_match.group(1).strip()
            data['ros_image'] = ros_image
            ros_version = ros_image.split(':')[-1]
            if ros_version.endswith('-ros-base'):
                ros_version = ros_version.split('-')[0]
            elif ros_version.endswith('-desktop-full'):
                ros_version = ros_version.split('-')[0]
            data['ros_version'] = ros_version
            ros1_distros = ["noetic", "melodic", "kinetic"]
            data['is_ros1'] = ros_version in ros1_distros
        else:
            print("Error: ROS_IMAGE information not found.")
            return None

        # [APT_PACKAGES]の抽出
        apt_packages_match = re.search(r"APT_PACKAGES:(.+?)GIT_REPOSITORIES:", content, re.DOTALL)
        if apt_packages_match:
            apt_packages = re.findall(r"- (.+)", apt_packages_match.group(1))
            data['apt_packages'] = [pkg.strip() for pkg in apt_packages]
        else:
            data['apt_packages'] = []

        # [GIT_REPOSITORIES]の抽出 (連番ID形式に対応)
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
                repo['name'] = repo_id # 連番IDを格納

                # url, branchを正規表現で抽出
                url_match = re.search(r"url: (.+)", section_content)
                branch_match = re.search(r"branch: (.+)", section_content)

                repo['url'] = url_match.group(1).strip() if url_match else None
                # ブランチが'(none)'でない場合にのみbranchを格納
                repo['branch'] = branch_match.group(1).strip() if branch_match and branch_match.group(1).strip() != '(none)' else None

                if repo['url']: # URLがある場合のみ追加
                    git_repos.append(repo)

        data['git_repos'] = git_repos

        return data

    except FileNotFoundError:
        print(f"Error: '{file_path}' not found. Please check the file name.")
        return None
    except Exception as e:
        print(f"Error: An error occurred during parsing: {e}")
        return None

def generate_dockerfile(data):
    """抽出したデータからDockerfileを生成する"""
    ros_distro = data['ros_version']
    is_ros1 = data['is_ros1']

    # ROS1かROS2かによってワークスペース名を決定
    workspace_name = "catkin_ws" if is_ros1 else "colcon_ws"

    # Gitリポジトリのクローンコマンドを生成
    git_clone_commands = ""
    for repo in data['git_repos']:
        if repo['url']:
            # branchが'main'でない場合にのみ-bオプションを付ける
            if repo['branch'] and repo['branch'] != 'main':
                git_clone_commands += f"RUN git clone -b {repo['branch']} {repo['url']} \n"
            else:
                git_clone_commands += f"RUN git clone {repo['url']} \n"

    # APTインストールコマンドの引数を整形
    apt_packages_str = " \\\n    ".join(data['apt_packages'])

    # bashrcに追記するセットアップファイルのパスを決定
    setup_path = "devel" if is_ros1 else "install"

    # ROS 1とROS 2で異なるビルドコマンドとrosdepコマンドを分岐
    if is_ros1:
        # ROS 1 (Catkin) 用のコマンドブロック
        build_commands = f"""# Build (execute after sourcing ROS environment)
RUN catkin init && \\
    bash -c "source /opt/ros/${{ROS_DISTRO}}/setup.bash && catkin build"
"""
        rosdep_block = f"""# Resolve dependencies
WORKDIR /root/{workspace_name}
# Initialize rosdep
RUN rosdep init || true && rosdep update
RUN rosdep install -r -y -i --from-paths /root/{workspace_name}/src
"""
    else: # ROS 2 (Colcon) 用のコマンドブロック
        build_commands = f"""# Build (execute after sourcing ROS environment)
RUN bash -c "source /opt/ros/${{ROS_DISTRO}}/setup.bash && colcon build"
"""
        rosdep_block = f"""# Resolve dependencies
WORKDIR /root/{workspace_name}
# Initialize rosdep
RUN apt-get update && rosdep update
RUN rosdep install -r -y -i --from-paths /root/{workspace_name}/src
"""

    # 環境変数ROS_DISTROにデスクトップ構成のサフィックスを含める
    ros_distro_env = ros_distro
    if data['ros_image'].endswith('-desktop-full'):
        ros_distro_env = f"{ros_distro}-desktop-full"

    # Dockerfile全体の文字列をf-stringで生成
    dockerfile_content = f"""# Base image: {data['ros_image']}
FROM {data['ros_image']}

# Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO={ros_distro_env}

# Install required packages
RUN apt-get update && apt-get install -y \\
    {apt_packages_str} \\
    && rm -rf /var/lib/apt/lists/*

# Create workspace
RUN mkdir -p /root/{workspace_name}/src
WORKDIR /root/{workspace_name}/src

# Clone required repositories
{git_clone_commands}
{rosdep_block}
{build_commands}

# Add environment variables to bashrc
RUN echo "source /opt/ros/${{ROS_DISTRO}}/setup.bash" >> /root/.bashrc && \\
    echo "source /root/{workspace_name}/{setup_path}/setup.bash" >> /root/.bashrc

# Set default working directory
WORKDIR /root/{workspace_name}

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
