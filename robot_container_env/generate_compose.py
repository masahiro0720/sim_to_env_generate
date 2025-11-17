import os
import re
import yaml  # pip install pyyaml
import sys
import shutil

# --- FastDDS XML Content ---
FASTDDS_XML_CONTENT = """<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS/fastRTPS_profiles.xsd">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>udpv4_transport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="base_profile" is_default_profile="true">
        <rtps>
            <builtin>
                <discovery_config>
                    <discoveryProtocol>SIMPLE</discoveryProtocol>
                    <leaseDuration>
                        <sec>20</sec>
                    </leaseDuration>
                    <initialAnnouncements>
                        <count>5</count>
                    </initialAnnouncements>
                </discovery_config>
            </builtin>
        </rtps>
    </participant>
</profiles>
"""

# --- Middleware Analysis Regex ---
ROS2_PATTERN = re.compile(r'FROM\s+ros:(humble|foxy|galactic|rolling|jazzy|iron)', re.IGNORECASE)
ROS1_PATTERN = re.compile(r'FROM\s+ros:(noetic|melodic|kinetic)', re.IGNORECASE)
UBUNTU_PATTERN = re.compile(r'FROM\s+ubuntu:', re.IGNORECASE)


def find_dockerfiles(dockerfile_dir):
    """Search for .Dockerfile files in the specified directory"""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    dockerfile_path_abs = os.path.join(script_dir, dockerfile_dir)

    if not os.path.isdir(dockerfile_path_abs):
        print(f"❌ Error: Directory not found: {dockerfile_path_abs}")
        print("Please check if the script is placed in the 'robot_container_env' directory.")
        return [], None
        
    files = [f for f in os.listdir(dockerfile_path_abs) if f.endswith(".Dockerfile")]
    return files, dockerfile_path_abs

def select_dockerfiles(files, integration_mode):
    """Let the user select Dockerfiles"""
    if not files:
        print("ℹ️ No .Dockerfile files found in the 'Dockerfile' directory.")
        return []

    print("📁 Found Dockerfiles:")
    for i, f in enumerate(files):
        print(f"  [{i+1}] {f}")
    
    print("\n✅ Select the Dockerfile numbers to include in docker-compose.yml.")
    print("   (e.g., 1,3  or  all)")
    
    if integration_mode:
        print("   (Integration Mode: Select 2 or more)")
    
    while True:
        try:
            choice = input("> ").strip().lower()
            if not choice:
                print("⚠️ No input. Aborting.")
                return []
                
            if choice == "all":
                selected_files = files
            else:
                selected_indices = [int(x.strip()) - 1 for x in choice.split(',')]
                selected_files = []
                
                for i in selected_indices:
                    if 0 <= i < len(files):
                        if files[i] not in selected_files:
                            selected_files.append(files[i])
                    else:
                        raise ValueError(f"Invalid number: {i+1}")
            
            if not selected_files:
                print("⚠️ Please select at least one.")
                continue

            if integration_mode and len(selected_files) < 2:
                print(f"❌ Error: Integration mode requires selecting 2 or more Dockerfiles.")
                print("Aborting.")
                return []
            
            return selected_files
        except ValueError as e:
            print(f"❌ Invalid input: {e}. Please enter numbers separated by commas (e.g., 1,2).")

def analyze_middleware(dockerfile_path_abs, filename):
    """Read Dockerfile (FROM line) to determine middleware"""
    try:
        full_path = os.path.join(dockerfile_path_abs, filename)
        if not os.path.exists(full_path):
            return "Error (File not found)"
            
        with open(full_path, 'r', encoding='utf-8') as f:
            content = f.read()

        if ROS2_PATTERN.search(content):
            return "ROS2"
        if ROS1_PATTERN.search(content):
            return "ROS1"
        if UBUNTU_PATTERN.search(content) and "RTM" in filename.upper():
            return "OpenRTM"
        
        # Fallback analysis
        if re.search(r'ROS_DISTRO=(humble|foxy|galactic|rolling|jazzy|iron)', content, re.IGNORECASE):
            return "ROS2"
        if re.search(r'ROS_DISTRO=(noetic|melodic|kinetic)', content, re.IGNORECASE):
             return "ROS1"
        if re.search(r'openrtm|rtm-aist', content, re.IGNORECASE):
            return "OpenRTM"
        
        return "Unknown"
    except Exception as e:
        print(f"⚠️ Error reading {filename}: {e}")
        return "Error"

def get_next_compose_dir_path(base_dir):
    """Determines the next 'compose_verX' directory path"""
    ver_pattern = re.compile(r'^compose_ver(\d+)$')
    max_ver = 0
    try:
        entries = os.listdir(base_dir)
        for entry in entries:
            if os.path.isdir(os.path.join(base_dir, entry)):
                match = ver_pattern.match(entry)
                if match:
                    ver = int(match.group(1))
                    if ver > max_ver:
                        max_ver = ver
    except FileNotFoundError:
        pass
    
    next_ver = max_ver + 1
    new_dir_name = f"compose_ver{next_ver}"
    return os.path.join(base_dir, new_dir_name)

def create_docker_compose(selected_files, context_path, dockerfile_path_abs, integration_mode):
    """Generate the content for docker-compose.yml"""
    
    print("\n🔬 Analyzing middleware for selected Dockerfiles:")
    middlewares = {}
    all_ros2 = True
    
    for f in selected_files:
        mw = analyze_middleware(dockerfile_path_abs, f)
        print(f"  - {f}: {mw}")
        middlewares[f] = mw
        if mw != "ROS2":
            all_ros2 = False
    
    apply_ros2_integration = integration_mode and all_ros2
    
    if integration_mode:
        if apply_ros2_integration:
            print("ℹ️ Integration Mode: [All ROS2] detected. Adding Fast DDS settings.")
        else:
            print("ℹ️ Integration Mode: [Mixed Middleware] detected. No extra settings needed.")

    def get_base_template():
        template = {
            'privileged': True,
            'devices': ["/dev/bus/usb:/dev/bus/usb"],
            'network_mode': "host",
            'environment': [
                'DISPLAY=${DISPLAY}',
                'QT_X11_NO_MITSHM=1'
            ],
            'volumes': ['/tmp/.X11-unix:/tmp/.X11-unix'],
            'tty': True,
            'stdin_open': True,
            'command': ['bash'],
            'restart': 'unless-stopped'
        }
        
        if apply_ros2_integration:
            template['ipc'] = "host"
            template['environment'].append('FASTRTPS_DEFAULT_PROFILES_FILE=/root/fastdds_profile.xml')
            template['volumes'].append('./fastdds_profile.xml:/root/fastdds_profile.xml')
        
        return template

    services = {}
    for i, dockerfile_name in enumerate(selected_files):
        service_name = f"module{i+1}"
        
        # --- Naming Logic ---
        if dockerfile_name == "MikataArm_ROS_open_manipulator_ROS_noetic-ros-base_profile.Dockerfile":
             base_name = "MikataArm_ROS_open_manipulator_ROS_noetic_ros_base"
        elif dockerfile_name == "MikataArmRTC_RTM_openmanipulator_RTM_18.04_profile.Dockerfile":
             base_name = "MikataArmRTC_RTM_openmanipulator_RTM_18.04_profile"
        else:
             base_name = dockerfile_name.removesuffix(".Dockerfile")
             base_name = base_name.replace("-", "_")

        # ★★★ 変更点: すべて小文字に変換 ★★★
        lowercase_base_name = base_name.lower()
        image_name = f"{lowercase_base_name}_image"
        container_name = f"{lowercase_base_name}_container"
        # ★★★ ここまで ★★★

        template_copy = get_base_template()

        service_data = {
            'build': {
                'context': context_path,
                'dockerfile': dockerfile_name
            },
            'image': image_name,
            'container_name': container_name,
            **template_copy
        }
        services[service_name] = service_data
        
    compose_data = {
        'services': services
    }
    
    return compose_data, apply_ros2_integration

def save_yaml(data, target_dir, output_filename):
    """Save the YAML data to the target directory"""
    
    class NoAliasDumper(yaml.Dumper):
        def ignore_aliases(self, data):
            return True

    try:
        output_path = os.path.join(target_dir, output_filename)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            yaml_str = yaml.dump(data, 
                                Dumper=NoAliasDumper, 
                                sort_keys=False, 
                                default_flow_style=False, 
                                indent=2, 
                                allow_unicode=True)
            
            # Re-insert English comments
            yaml_str = yaml_str.replace("privileged: true", "# --- [Req 1] Enable All USB Devices ---\n    privileged: true")
            yaml_str = yaml_str.replace("network_mode: host", "# --- [Req 2] Use Host Network ---\n    network_mode: host")
            yaml_str = yaml_str.replace("environment:", "# --- [Req 3] Enable GUI (X11) Forwarding ---\n    environment:")
            yaml_str = yaml_str.replace("tty: true", "# --- [Req 4] Start with Interactive Terminal (bash) ---\n    tty: true")
            yaml_str = yaml_str.replace("restart: unless-stopped", "# --- (Optional) Auto-Restart Policy ---\n    restart: unless-stopped")
            yaml_str = yaml_str.replace("ipc: host", "# --- [ROS2 Integration] Share IPC Namespace ---\n    ipc: host")
            
            f.write(yaml_str)
            
        print(f"\n🎉 Success! `{output_path}` has been generated.")
        return True
    except Exception as e:
        print(f"❌ Error: Failed to save `{output_filename}`: {e}")
        return False

def copy_dockerfiles(selected_files, source_dir_abs, target_dir):
    """Copy selected Dockerfiles to the new directory"""
    print(f"📄 Copying Dockerfiles to `{target_dir}`...")
    try:
        for filename in selected_files:
            source_path = os.path.join(source_dir_abs, filename)
            dest_path = os.path.join(target_dir, filename)
            shutil.copy(source_path, dest_path)
            print(f"  -> {filename}")
        return True
    except Exception as e:
        print(f"❌ Error: Failed to copy Dockerfiles: {e}")
        return False

def create_fastdds_profile(target_dir):
    """Create fastdds_profile.xml in the target directory"""
    output_path = os.path.join(target_dir, "fastdds_profile.xml")
    try:
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(FASTDDS_XML_CONTENT)
        print(f"📄 Generated Fast DDS profile (`fastdds_profile.xml`).")
        return True
    except Exception as e:
        print(f"❌ Error: Failed to generate `fastdds_profile.xml`: {e}")
        return False

# --- Main Process ---
def main():
    args = sys.argv[1:]
    integration_mode = False
    
    if '-f' in args and 'Integral' in args:
        try:
            f_index = args.index('-f')
            integral_index = args.index('Integral')
            if f_index < integral_index:
                 integration_mode = True
        except ValueError:
            pass
            
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    DOCKERFILE_DIR = "./Dockerfile"
    CONTEXT_PATH = "." 
    OUTPUT_FILENAME = "docker-compose.yml"
    
    # Output directory for compose_verX folders
    COMPOSE_BASE_DIR = os.path.join(SCRIPT_DIR, "compose_files")

    print("--- 🤖 Docker Compose Auto-Generator ---")
    if integration_mode:
        print("--- 🚀 Running in Integration Mode (-f Integral) ---")
    
    files, dockerfile_path_abs = find_dockerfiles(DOCKERFILE_DIR)
    
    if not files:
        print("Exiting.")
        sys.exit(1)
        
    selected_files = select_dockerfiles(files, integration_mode)
    if not selected_files:
        print("Exiting.")
        return
        
    print(f"\n🔧 Processing {len(selected_files)} Dockerfile(s):")
    for f in selected_files:
        print(f"  -> {f}")

    # 1. Ensure the 'compose_files' directory exists
    try:
        os.makedirs(COMPOSE_BASE_DIR, exist_ok=True)
    except OSError as e:
        print(f"❌ Error: Failed to create base directory: {COMPOSE_BASE_DIR}: {e}")
        return

    # 2. Get the next versioned path inside 'compose_files'
    new_compose_dir = get_next_compose_dir_path(COMPOSE_BASE_DIR)

    print(f"\n🆕 Creating new directory: {new_compose_dir}")
    try:
        os.makedirs(new_compose_dir)
    except OSError as e:
        print(f"❌ Error: Failed to create directory: {e}")
        return

    compose_data, apply_ros2_integration = create_docker_compose(
        selected_files, CONTEXT_PATH, dockerfile_path_abs, integration_mode
    )
    
    if not save_yaml(compose_data, new_compose_dir, OUTPUT_FILENAME):
        return

    if not copy_dockerfiles(selected_files, dockerfile_path_abs, new_compose_dir):
        return
    
    if apply_ros2_integration:
        if not create_fastdds_profile(new_compose_dir):
            return
    
    print("\n✅ All tasks completed.")
    print(f"   Generated directory: {new_compose_dir}")

if __name__ == "__main__":
    main()
