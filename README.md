# sim_to_env_generate

ISO22166-202規格に準拠したロボットソフトウェア情報モデル（SIM）から、Dockerベースの開発環境（DockerfileおよびDocker Compose）を自動生成するためのツールキット。

このリポジトリは、ロボットアプリケーションの環境構築を自動化し、再現性を高めることを目的としています。

## プロジェクト概要

このツールキットは、以下の3つの主要なステップ（ディレクトリ）で構成されています。

1.  **SIM_generator (GUI):**
    GUIを使用して、ロボットの仕様を定義した情報モデル（`.xml`）を作成します。
2.  **Dockerfile_generator:**
    `1` で作成した `.xml` を読み込み、ROSやOpenRTMの環境を構築する `Dockerfile` を自動生成します。
3.  **robot_container_env:**
    `2` で生成された `Dockerfile` 群を元に、複数のコンテナを協調させる `docker-compose.yml` を自動生成します。

---

## 実行環境の構築 (セットアップ)

このリポジトリの全機能を利用するためのセットアップ手順です。

1.  **リポジトリのクローン**
    ```bash
    git clone [https://github.com/masahiro0720/sim_to_env_generate.git](https://github.com/masahiro0720/sim_to_env_generate.git)
    cd sim_to_env_generate
    ```

2.  **Python仮想環境の作成と有効化**
    * `SIM_generator` ディレクトリ内で仮想環境を作成します。
    ```bash
    cd SIM_generator
    python3 -m venv venv
    source venv/bin/activate
    ```

3.  **必要ライブラリの一括インストール**
    * 必要なライブラリ（Streamlit, PyYAML）をインストールします。
    ```bash
    # (venv) が有効化されていることを確認
    
    # SIM_generator (GUI) の依存関係をインストール
    (venv)$ pip install -r requirements.txt
    
    # generate_compose.py の依存関係をインストール
    (venv)$ pip install pyyaml
    ```
    
    これでセットアップは完了です。

---

## 使用方法

セットアップが完了したら、以下の3ステップでコンテナ環境を生成できます。
**（作業は `venv` を有効化したターミナルで続けてください）**

### Step 1: SIM (情報モデル) の作成 (`SIM_generator/`)

まず、GUIを使用してロボットソフトウェアの仕様を定義する `.xml` ファイルを生成します。

1.  **GUIの起動**
    * セットアップ直後は既に `SIM_generator` ディレクトリにいます。
    ```bash
    # (venv) が有効化されていることを確認
    (venv)$ streamlit run sim_generator.py
    ```

2.  **ブラウザでアクセス**
    * ターミナルに表示される `Local URL`（通常は `http://localhost:8501`）にブラウザでアクセスします。

3.  **SIMの作成と保存**
    * GUIの指示に従い、情報モデルの各項目を入力します。
    * 完成したら、`.xml` ファイルをエクスポート（ダウンロードまたはサーバー保存）します。

> **推奨:**
> 生成された `.xml` ファイルは、プロジェクトルートの **`SIM/`** ディレクトリに保存してください。Step 2のスクリプトは、デフォルトでこのディレクトリを参照します。

### Step 2: Dockerfile の自動生成 (`Dockerfile_generator/`)

次に、作成した `.xml` ファイルを元に `Dockerfile` を生成します。

1.  **生成スクリプトの実行**
    * `Generate_all_file.py` を実行します。
    ```bash
    # (venv) が有効化されていることを確認
    (venv)$ cd ../Dockerfile_generator  # ディレクトリを移動
    (venv)$ python3 Generate_all_file.py [オプション]
    ```

2.  **SIMの選択**
    * スクリプトが `SIM/` ディレクトリ内の `.xml` ファイルを検出し、リスト表示します。
    * `Dockerfile` を生成したい情報モデルの番号を入力してください。

3.  **出力**
    * 生成された `Dockerfile` は **`robot_container_env/Dockerfile/`** ディレクトリ内に保存されます。

#### `Generate_all_file.py` のオプション

| 引数 | 説明 |
| :--- | :--- |
| (なし) | 最小構成 (`min`) かつ最新バージョンで `Dockerfile` を生成します。（デフォルト）|
| `-f min` | ROSやOpenRTMの**最小限**のツール（`ros-base` 等）で環境を構築します。|
| `-f full` | Rviz, MoveIt, Gazeboなどを含む**フルパッケージ**で環境を構築します。|
| `-f version` | ROSのバージョン（Humble, Jazzy等）やOpenRTMのUbuntuベースバージョンを選択するプロンプトが表示されます。|

### Step 3: Docker Compose の自動生成 (`robot_container_env/`)

最後に、複数の `Dockerfile` を組み合わせて、システム全体を起動するための `docker-compose.yml` を生成します。

1.  **生成スクリプトの実行**
    * `generate_compose.py` を実行します。
    ```bash
    # (venv) が有効化されていることを確認
    (venv)$ cd ../robot_container_env  # ディレクトリを移動
    (venv)$ python3 generate_compose.py [オプション]
    ```

2.  **Dockerfileの選択**
    * スクリプトが `robot_container_env/Dockerfile/` 内の `Dockerfile` を検出し、リスト表示します。
    * Composeに含めたい `Dockerfile` の番号を（複数可）選択してください。

3.  **出力**
    * **`robot_container_env/compose_files/`** 内に、`compose_ver1`, `compose_ver2`, ... という名前で新しいディレクトリが作成されます。
    * この新ディレクトリ内に、生成された `docker-compose.yml` と、対象となった `Dockerfile` のコピーがセットで格納されます。

#### `generate_compose.py` のオプション

| 引数 | 説明 |
| :--- | :--- |
| `-f Integral` | 複数のROS 2コンテナを選択した場合、コンテナ間で正常に通信（トピック、サービス）を行うための設定を `docker-compose.yml` に追加します。|
