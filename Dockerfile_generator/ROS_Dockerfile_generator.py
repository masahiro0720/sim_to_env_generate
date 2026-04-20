import re
import os
import sys

def parse_profile_data(file_path):
    data = {}
    try:
        with open(file_path, 'r') as f:
            content = f.read()

        def get_val(key):
            m = re.search(f"{key}: (.+)", content)
            return m.group(1).strip() if m else None

        data['ros_image'] = get_val("ROS_IMAGE")
        data['ros_distro'] = get_val("ROS_DISTRO")

        m = re.search(r"COMPILER_NAME: (.+)", content)
        data['compiler_name'] = m.group(1).strip() if m else "GCC"

        data['workspace'] = get_val("WORKSPACE_NAME")

        # --- OCI Annotation用 メタデータの読み取り ---
        m = re.search(r"DESCRIPTION: (.*)", content)
        data['description'] = m.group(1).strip() if m else ""

        m = re.search(r"AUTHORS: (.*)", content)
        data['authors'] = m.group(1).strip() if m else ""
        # ---------------------------------------------

        m = re.search(r"APT_PACKAGES:(.+?)PIP_PACKAGES:", content, re.DOTALL)
        data['apt_packages'] = [x.strip() for x in re.findall(r"- (.+)", m.group(1))] if m else []

        m = re.search(r"PIP_PACKAGES:(.+?)GIT_REPOSITORIES:", content, re.DOTALL)
        data['pip_packages'] = [x.strip() for x in re.findall(r"- (.+)", m.group(1))] if m else []

        m = re.search(r"GIT_REPOSITORIES:(.+)", content, re.DOTALL)
        data['git_repos'] = []
        if m:
            block = m.group(1).strip()
            parts = re.split(r"-\s*(gitURL\d+)", block)
            for i in range(1, len(parts), 2):
                repo = {'name': parts[i].strip()}
                sec = parts[i+1]
                u = re.search(r"url: (.+)", sec)
                b = re.search(r"branch: (.+)", sec)
                repo['url'] = u.group(1).strip() if u else None
                bv = b.group(1).strip() if b else None
                repo['branch'] = bv if bv != '(none)' else None
                if repo['url']: data['git_repos'].append(repo)

        return data
    except Exception as e:
        print(f"Error parsing profile: {e}")
        return None

def generate_dockerfile(data):
    ros_distro = data['ros_distro']
    is_ros1 = ros_distro in ["kinetic", "melodic", "noetic"]
    ws_name = data.get('workspace') or ("catkin_ws" if is_ros1 else "colcon_ws")
    setup_path = "devel" if is_ros1 else "install"

    # --- OCI Reproducibility: Digest Mapping ---
    DIGEST_MAP = {
        "docker.io/library/ros:kinetic-ros-base": "@sha256:2d17462cc7867be439121a141b7145c22502693fa5c0df3b2c15949d05e257eb",
        "docker.io/osrf/ros:kinetic-desktop-full": "@sha256:d8c5581177de4106517208d17963df02bb91f21151df2f6f571597a7e3715c0e",
        "docker.io/library/ros:melodic-ros-base": "@sha256:db4c9f983086b51dfdfb0fa0bdc4873c52a0a2df8417c80517e4bb00cc01844b",
        "docker.io/osrf/ros:melodic-desktop-full": "@sha256:ec5b9b9716e4dbcd7d722e059f3d53fb79de09df07977463f69b82caef064ed3",
        "docker.io/library/ros:noetic-ros-base": "@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d",
        "docker.io/osrf/ros:noetic-desktop-full": "@sha256:14cb898430d62482ebc7c71c6b4bd05e2446e6de5ef18a53819e70fa69a91f5a",
        "docker.io/library/ros:foxy-ros-base": "@sha256:c22502693fa5c0df3b2c15949d05e257ebc7965922abedde992cbfffa2ce1da1",
        "docker.io/osrf/ros:foxy-desktop": "@sha256:a68b55d28b5f3a5796cb103c8b4fb7122a7f5a25c6e838b0fb60980bd32dbbd7",
        "docker.io/library/ros:humble-ros-base": "@sha256:b652da5e5108f7bd8f48df270498b6727282e70eec1d87e0ce3ba25ee6796d11",
        "docker.io/osrf/ros:humble-desktop-full": "@sha256:a7809549a9019d370d953c310df5a9b66faf61cfa49e5b8bf08a0bdf67496026",
        "docker.io/library/ros:jazzy-ros-base": "@sha256:cf4a86b16954f9d2d09c2a632ed6c546e9dfd6e74ab8672cbcc09c0a6db8b21c",
        "docker.io/osrf/ros:jazzy-desktop-full": "@sha256:1044439c05bd9a8ff4b0c9f134dc282b0e611e97d0baae7b203c9e6bb07df74b"
    }
    ros_image = data['ros_image']
    base_image_with_digest = ros_image + DIGEST_MAP.get(ros_image, "")

    apt_packages_str = " \\\n    ".join(data['apt_packages'])

    pip_block = ""
    if data['pip_packages']:
        pip_pkgs = " ".join(data['pip_packages'])
        pip_block = f"\n# Install Python dependencies\nRUN pip3 install --no-cache-dir {pip_pkgs}\n"

    git_cmds = ""
    for repo in data['git_repos']:
        b_opt = f"-b {repo['branch']} " if repo['branch'] else ""
        git_cmds += f"RUN git clone {b_opt}{repo['url']}\n"

    if is_ros1:
        build_cmd = f"""RUN catkin init && bash -c "source /opt/ros/${{ROS_DISTRO}}/setup.bash && catkin build" """
        rosdep_cmd = f"""RUN rosdep init || true && rosdep update && sudo apt-get update && rosdep install -r -y -i --from-paths src"""
    else:
        build_cmd = f"""RUN bash -c "source /opt/ros/${{ROS_DISTRO}}/setup.bash && colcon build --symlink-install" """
        rosdep_cmd = f"""RUN sudo apt-get update && rosdep update && rosdep install -r -y -i --from-paths src"""

    oci_labels = ""
    desc_safe = data.get('description', '').replace('"', '\\"').replace('\n', ' ')
    auth_safe = data.get('authors', '').replace('"', '\\"')

    if desc_safe or auth_safe:
        oci_labels = "\n# OCI Annotations (Image Metadata)\n"
        if auth_safe:
            oci_labels += f'LABEL org.opencontainers.image.authors="{auth_safe}"\n'
        if desc_safe:
            oci_labels += f'LABEL org.opencontainers.image.description="{desc_safe}"\n'

    return f"""# Base image (Pinned by SHA256 Digest for Reproducibility)
FROM {base_image_with_digest}

# --- OCI Multi-Architecture Support ---
ARG TARGETARCH
LABEL org.opencontainers.image.architecture="${{TARGETARCH:-amd64}}"{oci_labels}
# --------------------------------------

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO={ros_distro}

# Install APT packages and sudo
RUN apt-get update && apt-get install -y \\
    sudo \\
    {apt_packages_str} \\
    && rm -rf /var/lib/apt/lists/*

{pip_block}

# --- OCI Security: Create non-root user (rosuser) ---
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \\
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \\
    && echo $USERNAME ALL=\\(root\\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \\
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to non-root user
USER $USERNAME

# --- UX: Automatic source for Interactive Shell (exec) ---
RUN echo "source /opt/ros/${{ROS_DISTRO}}/setup.bash" >> /home/$USERNAME/.bashrc && \\
    echo "if [ -f /workspace/{ws_name}/{setup_path}/setup.bash ]; then source /workspace/{ws_name}/{setup_path}/setup.bash; fi" >> /home/$USERNAME/.bashrc
# ---------------------------------------------------

# Setup Workspace
RUN sudo mkdir -p /workspace/{ws_name}/src \\
    && sudo chown -R $USERNAME:$USERNAME /workspace
WORKDIR /workspace/{ws_name}/src

# Clone & Build
{git_cmds}
WORKDIR /workspace/{ws_name}
{rosdep_cmd}
{build_cmd}

# --- OCI Execution Control: ENTRYPOINT ---
# Create entrypoint script
RUN echo '#!/bin/bash' > /workspace/entrypoint.sh && \\
    echo 'set -e' >> /workspace/entrypoint.sh && \\
    echo 'source /opt/ros/${{ROS_DISTRO}}/setup.bash' >> /workspace/entrypoint.sh && \\
    echo 'if [ -f /workspace/{ws_name}/{setup_path}/setup.bash ]; then source /workspace/{ws_name}/{setup_path}/setup.bash; fi' >> /workspace/entrypoint.sh && \\
    echo 'exec "$@"' >> /workspace/entrypoint.sh && \\
    sudo chmod +x /workspace/entrypoint.sh

WORKDIR /workspace/{ws_name}

# --- OCI State Management: VOLUME ---
# Fix Errno 13: Pre-create the log directory and assign ownership to rosuser before mounting
RUN mkdir -p /home/$USERNAME/.ros/log \
    && sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/.ros
VOLUME ["/home/$USERNAME/.ros/log"]
# ------------------------------------

STOPSIGNAL SIGINT
ENTRYPOINT ["/workspace/entrypoint.sh"]
CMD ["bash"]
"""

if __name__ == "__main__":
    if len(sys.argv) < 3: sys.exit(1)
    prof = sys.argv[1]
    out_dir = sys.argv[2]
    if os.path.exists(prof):
        print(f"--- Generating Dockerfile from {os.path.basename(prof)} ---")
        data = parse_profile_data(prof)
        if data:
            content = generate_dockerfile(data)
            base_name = os.path.basename(prof).replace('_profile.txt', '')
            out_path = os.path.join(out_dir, base_name + ".Dockerfile")
            with open(out_path, 'w') as f: f.write(content)
            print(f"Created: {out_path}")