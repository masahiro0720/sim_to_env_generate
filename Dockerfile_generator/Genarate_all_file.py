import subprocess
import os
import glob
import time
import sys

# --- ディレクトリパス定義 ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
# XML入力元
XML_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'SIM'))
# Dockerfile(.Dockerfile)出力先
DOCKERFILE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, '..', 'robot_container_env', 'Dockerfile'))
# プロファイル(.txt)出力先
TXT_FILES_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, 'txt_files'))
# ---

def run_script(script_name, args=None, wait_time=1, interactive=False):
    """
    指定されたPythonスクリプトを実行し、完了を待機するヘルパー関数。
    interactive=Trueの場合、リアルタイムで出力を表示し、ユーザー入力を許可する。
    interactive=Falseの場合、出力を捕捉して完了後にまとめて表示する。
    """
    print(f"--- Executing '{script_name}' ---")

    # 実行するスクリプトのフルパスを指定
    script_path = os.path.join(SCRIPT_DIR, script_name)
    if not os.path.exists(script_path):
        print(f"Error: '{script_path}' not found.")
        return False, None

    # スクリプトファイルに実行権限を付与
    os.chmod(script_path, 0o755)

    # 実行するコマンドをリストとして構築
    command = ['python3', script_path]
    if args:
        command.extend(args)

    try:
        if interactive:
            # 対話モード: 出力をリアルタイムで表示（ユーザー入力用）
            process = subprocess.run(command, text=True, check=True)
            print(f"--- Execution of '{script_name}' completed ---")
        else:
            # 非対話モード: 出力を捕捉して後でまとめて表示
            result = subprocess.run(command, capture_output=True, text=True, check=True)
            print(result.stdout)
            print(f"--- Execution of '{script_name}' completed ---")

        time.sleep(wait_time)
        return True, None
    except FileNotFoundError:
        print(f"Error: '{script_path}' not found. Check the file name and path.")
        return False, None
    except subprocess.CalledProcessError as e:
        print(f"Error: An error occurred during the execution of '{script_name}'.")
        if e.stderr:
            print(f"Details:\n{e.stderr}")
        else:
            print("Check the console output above for error details.")
        return False, None
    except KeyboardInterrupt:
        print("\nExecution interrupted by user.")
        return False, None
    except Exception as e:
        print(f"Unexpected error: {e}")
        return False, None

def find_latest_profile_file(directory):
    """指定されたディレクトリ内で、最も新しく更新された .txt ファイルを見つける"""
    pattern = os.path.join(directory, "*.txt")
    files = glob.glob(pattern)
    if not files:
        return None
    # ファイルの最終更新時刻で最新のファイルを見つける
    latest_file = max(files, key=os.path.getmtime)
    return latest_file

if __name__ == "__main__":
    print("--- Starting workflow automation script ---")

    # --- ディレクトリ存在チェック ---
    if not os.path.exists(XML_DIR):
        print(f"Error: XML directory not found at {XML_DIR}")
        sys.exit(1)
    if not os.path.exists(DOCKERFILE_DIR):
        print(f"Warning: Dockerfile directory not found at {DOCKERFILE_DIR}")
        print("Creating directory...")
        os.makedirs(DOCKERFILE_DIR)
    # txt_files ディレクトリの存在チェックと作成
    if not os.path.exists(TXT_FILES_DIR):
        print(f"Warning: txt_files directory not found at {TXT_FILES_DIR}")
        print("Creating directory...")
        os.makedirs(TXT_FILES_DIR)

    # --- XMLファイル選択 ---
    print(f"\nSearching for XML files in: {XML_DIR}")
    xml_files = sorted(glob.glob(os.path.join(XML_DIR, '*.xml')))
    
    if not xml_files:
        print(f"Error: No .xml files found in {XML_DIR}")
        sys.exit(1)

    print("Please select the XML file(s) to process:")
    for i, xml_file in enumerate(xml_files):
        print(f"  {i+1}: {os.path.basename(xml_file)}")
    print("  0: ALL files")

    selected_indices = []
    while True:
        try:
            user_input = input("Enter number(s) separated by comma (e.g., 1,3 or 0): ")
            if user_input.strip() == '0':
                selected_indices = list(range(len(xml_files)))
                break
            
            indices = [int(i.strip()) - 1 for i in user_input.split(',')]
            
            if all(0 <= i < len(xml_files) for i in indices):
                selected_indices = indices
                break
            else:
                print("Error: Invalid selection. Please enter numbers from the list.")
        except ValueError:
            print("Error: Invalid input. Please enter numbers.")

    xml_files_to_process = [xml_files[i] for i in selected_indices]
    print(f"Selected files: {[os.path.basename(f) for f in xml_files_to_process]}")

    # --- 引数処理 ---
    # 初期値: 最小構成('min') と 自動最新バージョン選択('latest')
    config_mode = 'min'
    version_select_mode = 'latest' # デフォルトは 'latest' (対話なし)

    # コマンドライン引数をパース
    if len(sys.argv) > 2 and (sys.argv[1] == '-f' or sys.argv[1] == '--config'):
        args_list = sys.argv[2:]
        args_set = set(args_list)

        # 構成モードの決定
        if 'full' in args_set:
            config_mode = 'full'
        elif 'min' in args_set:
            config_mode = 'min'

        # バージョン選択モードの決定
        if 'version_select' in args_set:
            version_select_mode = 'version_select' # 引数があれば対話ありに変更

        # 無効な引数がないかの簡易チェック
        valid_args = {'min', 'full', 'version_select'}
        if not args_set.issubset(valid_args):
            invalid_args = args_set - valid_args
            print(f"Error: Invalid argument(s) detected: {', '.join(invalid_args)}. Please specify 'min', 'full', or 'version_select'.")
            sys.exit(1)

    print(f"Configuration mode: {config_mode}")
    print(f"Version selection mode: {version_select_mode}")

    # --- スクリプト実行 ---

    # ステップ①: プロファイル生成スクリプトを実行
    profile_gen_script = "ROS_RTM_software_profile_generator_new.py"
    # 'version_select' が指定された時だけ interactive=True にする
    is_interactive = (version_select_mode == 'version_select')

    # 引数リスト: [config, version_mode, txt_output_dir, xml_path1, ...]
    profile_args = [config_mode, version_select_mode, TXT_FILES_DIR] + xml_files_to_process

    print(f"Running profile generator (Interactive: {is_interactive})...")
    success, _ = run_script(profile_gen_script, args=profile_args, interactive=is_interactive)

    if not success:
        print("Workflow aborted.")
    else:
        # ステップ②: 生成されたファイル名からミドルウェアを判定
        # .txt ファイルを TXT_FILES_DIR から検索
        latest_profile = find_latest_profile_file(TXT_FILES_DIR) 

        if not latest_profile:
            print(f"Error: Profile file (.txt) not found in {TXT_FILES_DIR}.")
        else:
            print(f"Latest profile found: {os.path.basename(latest_profile)}")
            dockerfile_gen_script = None
            if "_ROS_" in latest_profile.upper():
                print(f"Detected middleware: ROS. Dockerfile generator script: ROS_Dockerfile_generator.py")
                dockerfile_gen_script = "ROS_Dockerfile_generator.py"
            elif "_RTM_" in latest_profile.upper():
                print(f"Detected middleware: OpenRTM. Dockerfile generator script: RTM_Dockerfile_generator.py")
                dockerfile_gen_script = "RTM_Dockerfile_generator.py"
            else:
                print(f"Error: Could not determine middleware from profile file name '{latest_profile}'.")
                print("Please include '_ROS_' or '_RTM_' in the file name.")

            # ステップ③: 適切なDockerfile生成スクリプトを実行
            if dockerfile_gen_script:
                # Dockerfile生成スクリプトに渡す引数: [profile_file_path, output_dir_path]
                dockerfile_args = [latest_profile, DOCKERFILE_DIR]
                run_script(dockerfile_gen_script, args=dockerfile_args, interactive=False)
                print("--- Workflow completed successfully ---")
