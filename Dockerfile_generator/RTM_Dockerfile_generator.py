import re
import glob
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

        data['middleware_name'] = get_val("MIDDLEWARE_NAME")
        data['os_name'] = get_val("OS_NAME")
        data['os_version'] = get_val("OS_VERSION")
        data['compiler_name'] = get_val("COMPILER_NAME")
        data['compiler_version'] = get_val("COMPILER_VERSION")
        data['workspace'] = get_val("WORKSPACE_NAME")

        # --- OCI Annotation用 メタデータの読み取り ---
        data['description'] = get_val("DESCRIPTION") or ""
        data['authors'] = get_val("AUTHORS") or ""
        # ---------------------------------------------

        m = re.search(r"APT_PACKAGES:(.+?)PIP_PACKAGES:", content, re.DOTALL)
        if not m: m = re.search(r"APT_PACKAGES:(.+?)GIT_REPOSITORIES:", content, re.DOTALL)
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
        print(f"Error parsing RTM profile: {e}")
        return None

def generate_dockerfile(data):
    os_name = data['os_name'].lower()
    if os_name == 'linux': os_name = 'ubuntu'

    os_ver_raw = data['os_version']
    match = re.search(r'(\d+\.\d+)', os_ver_raw)
    os_ver = match.group(1) if match else os_ver_raw

    ws_name = data.get('workspace') or "workspace"

    codenames = {
        "24.04": "noble", "22.04": "jammy", "20.04": "focal",
        "18.04": "bionic", "16.04": "xenial"
    }
    codename = codenames.get(os_ver, "focal")

    base_image = f"docker.io/library/{os_name}:{os_ver}"

    # --- OCI Reproducibility: Digest Mapping (Verified for April 2026) ---
    DIGEST_MAP = {
        "docker.io/library/ubuntu:16.04": "@sha256:1f1a2d56de1d604801a9671f301190704c25d604a416f59e03c04f5c6ffee0d6",
        "docker.io/library/ubuntu:18.04": "@sha256:152dc042452c496007f07ca9127571cb9c29697f42acbfad72324b2bb2e43c98",
        "docker.io/library/ubuntu:20.04": "@sha256:8feb4d8ca5354def3d8fce243717141ce31e2c428701f6682bd2fafe15388214",
        "docker.io/library/ubuntu:22.04": "@sha256:a6d2b38300ce017add71440577d5b0a90460d0e57fd7aec21dc0d1f547a8ef1d",
        "docker.io/library/ubuntu:24.04": "@sha256:8a37d68f4f73ebf3d4efafbcf66379bf3728902a8038616808f04e34a9ab63ee"
    }

    base_image_with_digest = base_image + DIGEST_MAP.get(base_image, "")
    # -------------------------------------------

    try:
        ver_float = float(os_ver)
    except ValueError:
        ver_float = 20.04

    # --- 1. EOL (End of Life) 対策 ---
    eol_fix = ""
    if ver_float < 18.04:
        eol_fix = """
# Fix for EOL (End Of Life) Ubuntu versions
RUN sed -i -r 's/([a-z]{2}\\.)?archive.ubuntu.com/old-releases.ubuntu.com/g' /etc/apt/sources.list \\
    && sed -i -r 's/security.ubuntu.com/old-releases.ubuntu.com/g' /etc/apt/sources.list
"""

    repo_setup = ""
    if ver_float >= 22.04:
        repo_setup = "# OpenRTM official repository skipped for newer Ubuntu (Using OS standard packages)"
    else:
        repo_url = f"deb [trusted=yes] http://openrtm.org/pub/Linux/ubuntu {codename} main"
        repo_setup = f"""# OpenRTM Repo & APT
RUN rm -f /etc/apt/sources.list.d/openrtm.list \\
    && echo "{repo_url}" > /etc/apt/sources.list.d/openrtm.list"""

    apt_packages_str = " \\\n    ".join(data['apt_packages'])

    pip_block = ""
    if data['pip_packages']:
        pip_pkgs = " ".join(data['pip_packages'])
        pip_block = f"""
# Install Python dependencies
RUN pip3 install --no-cache-dir {pip_pkgs}
"""

    is_python_rtm = (data.get('compiler_name') == 'Python')
    setuptools_fix = ""
    if is_python_rtm:
        if ver_float < 22.04:
            setuptools_fix = """
# Upgrade pip and Fix setuptools version for older Python compatibility
RUN pip3 install --upgrade pip && pip3 install "setuptools<60.0.0" wheel
"""
        else:
            setuptools_fix = """
# Upgrade pip
RUN pip3 install --upgrade pip setuptools wheel
"""

    build_commands = ""
    for repo in data['git_repos']:
        rname = repo['url'].split('/')[-1].replace('.git', '')
        b_opt = f"-b {repo['branch']} " if repo['branch'] else ""
        repo_url = repo['url']

        if is_python_rtm:
            build_commands += f"""
# Clone and Install Python Component: {rname}
RUN git clone {b_opt}{repo_url} \\
    && cd {rname} \\
    && if [ -f requirements.txt ]; then sudo pip3 install -r requirements.txt; fi \\
    && if [ -f setup.py ]; then sudo python3 setup.py install; fi
"""
        else:
            build_commands += f"""
# Clone and Build C++ RTC: {rname}
RUN git clone {b_opt}{repo_url} \\
    && cd {rname} \\
    && git submodule update --init --recursive \\
    && mkdir -p build && cd build \\
    && cmake .. -DBUILD_DOCUMENTATION=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \\
    && make -j$(nproc)
"""

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

# Install sudo first
RUN apt-get update && apt-get install -y sudo && rm -rf /var/lib/apt/lists/*

{eol_fix}

{repo_setup}

# Install packages
RUN apt-get update \\
    && apt-get install -y --allow-unauthenticated \\
    {apt_packages_str} \\
    && apt-get clean \\
    && rm -rf /var/lib/apt/lists/*

{pip_block}

{setuptools_fix}

# --- OCI Security: Create non-root user (rtmuser) ---
ARG USERNAME=rtmuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \\
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \\
    && echo $USERNAME ALL=\\(root\\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \\
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN mkdir -p /workspace/{ws_name} && chown -R $USERNAME:$USERNAME /workspace
USER $USERNAME
WORKDIR /workspace/{ws_name}
# ---------------------------------------------------

{build_commands}

# --- OCI Execution Control: ENTRYPOINT ---
# Create entrypoint script
RUN echo '#!/bin/bash' > /workspace/entrypoint.sh && \\
    echo 'set -e' >> /workspace/entrypoint.sh && \\
    echo '# Add RTM specific environment variables here if needed in the future' >> /workspace/entrypoint.sh && \\
    echo 'exec "$@"' >> /workspace/entrypoint.sh && \\
    sudo chmod +x /workspace/entrypoint.sh

# RTM用のログディレクトリを作成
RUN mkdir -p /workspace/{ws_name}/logs && sudo chown $USERNAME:$USERNAME /workspace/{ws_name}/logs

WORKDIR /workspace/{ws_name}

# --- OCI State Management: VOLUME ---
VOLUME ["/workspace/{ws_name}/logs"]
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
        print(f"--- Generating RTM Dockerfile from {os.path.basename(prof)} ---")
        data = parse_profile_data(prof)
        if data:
            content = generate_dockerfile(data)
            base_name = os.path.basename(prof).replace('_profile.txt', '')
            out_path = os.path.join(out_dir, base_name + ".Dockerfile")
            with open(out_path, 'w') as f: f.write(content)
            print(f"Created: {out_path}")