不需要每次都完整重新安装。

OpenPLC Runtime v4 的设计架构允许你根据更新内容的类型，选择最轻量化的更新方式。以下是四种常见的更新场景及对应操作：

### 1. 仅更新控制器逻辑（最频繁）
如果你只是在 OpenPLC Editor 里改了梯形图或 ST 代码，**完全不需要重新运行 install.sh**。
*   **操作**：直接在 Editor 点击“传输并运行” (Upload)。
*   **原理**：Runtime 会自动接收 ZIP 包、在本地 `build` 目录重新编译生成一个新的 `.so` 库并动态加载。

### 2. 仅更新 Python 服务代码（Webserver / REST API）
如果你通过 `git pull` 更新了 webserver 文件夹下的 Python 脚本，且没有新增第三方依赖：
*   **操作**：重启服务即可。
    ```bash
    sudo systemctl restart openplc-runtime
    # 或者手动重启
    sudo ./start_openplc.sh
    ```

### 3. 更新了 C++ 核心代码（PLC Core）
如果你看到 src 目录下有代码变动：
*   **操作**：手动重新编译核心，无需重装 Python 依赖。
    ```bash
    cd build
    cmake ..
    make -j$(nproc)
    ```

### 4. 需要完整更新（新增了 Python 依赖或大幅度重构）
当你看到 requirements.txt 有变化，或者官方发布了重大版本升级（如你这次从 v4.1.3 升到 v4.1.5）：
*   **操作**：建议重新运行 install.sh。
*   **优化**：因为你的环境已经是离线的，你可以继续复用之前的参数：
    ```bash
    export SKIP_DEPS=1
    export PYTHON_BIN=/home/ld/OpenPLC/python310/bin/python3.10
    # 如果有新包，记得在宿主机拉取新的 wheel 放入 offline_wheels
    export PIP_FIND_LINKS=$(pwd)/offline_wheels
    sudo -E ./install.sh
    ```

---

### 总结：什么时候**必须**重新安装？
*   修改了 requirements.txt。
*   修改了 CMakeLists.txt。
*   新增了插件（Plugin）。
*   更换了 Python 解释器路径。

**日常开发建议**：只要服务能跑起来，优先尝试 `git pull` + `systemctl restart`。只有报错提示缺包或编译失败时，才去跑 install.sh。