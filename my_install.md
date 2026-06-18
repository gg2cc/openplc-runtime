# OpenPLC Runtime v4 离线部署指南 (RK3562 / Python 3.10)

本指南总结了在 RK3562 (Ubuntu 20.04 ARM64) 平台上，针对 Read-Only 文件系统环境，使用自定义 Python 3.10 路径进行离线安装的过程。

## 1. 宿主机准备 (联网环境)

在与目标机架构相同（或使用 `pip download` 指定架构）的机器上准备依赖包。

```bash
# 克隆仓库
cd /home/ld/OpenPLC/openplc-runtime

# 下载 Python 依赖包 (Wheels) 到 wheelhouse 目录
# 1) 聚合主程序与所有插件的依赖清单 (确保换行，防止内容粘连)
for f in requirements.txt core/src/drivers/plugins/python/*/requirements.txt; do
    cat "$f"; echo ""
done | grep -v "^#" | grep -v "^$" | sort -u > all-req.txt

# 2) 准备离线包目录
BUNDLE=/home/ld/OpenPLC/openplc-offline-bundle-v4.1.5
rm -rf "$BUNDLE"
mkdir -p "$BUNDLE"/{src,wheelhouse,debs}

# 3) 打源码包（不含 .git）
tar --exclude=.git -czf "$BUNDLE/src/openplc-runtime-v4.1.5.tar.gz" .

# 4) 打包 python3.10
mkdir -p "$BUNDLE/python_env"
# 将你的 Python 3.10 拷贝进去
# 注意：使用 -a 保留软链接和权限
cp -a /home/ld/OpenPLC/python310 "$BUNDLE/python_env/"

# 5) 下载 ARM64 (RK3562) 且适配 Python 3.10 的离线 Wheel 包

第一步：先不带 --only-binary 下载，让 pip 允许处理源码包同步

python3 -m pip download \
  -r all-req.txt \
  -d "$BUNDLE/wheelhouse" \
  --platform manylinux2014_aarch64 \
  --implementation cp \
  --python-version 3.10 \
  --abi cp310

python3 -m pip download \
  "wait-for2>=0.4.1" \
  -d "$BUNDLE/wheelhouse" \
  --platform manylinux2014_aarch64 \
  --implementation cp \
  --python-version 3.10 \
  --abi cp310

cd /home/ld/OpenPLC/openplc-runtime

# 明确下载这三个工具的 ARM64 版本
python3 -m pip download \
  pip setuptools wheel \
  -d "/home/ld/OpenPLC/openplc-offline-bundle-v4.1.5/wheelhouse" \
  --platform manylinux2014_aarch64 \
  --only-binary=:all: \
  --implementation cp \
  --python-version 3.10 \
  --abi cp310

cd /home/ld/OpenPLC/openplc-runtime
BUNDLE=/home/ld/OpenPLC/openplc-offline-bundle-v4.1.5

# 明确下载满足要求的 filelock 版本
python3 -m pip download \
  "filelock>=3.24.2,<4" \
  -d "$BUNDLE/wheelhouse" \
  --platform manylinux2014_aarch64 \
  --only-binary=:all: \
  --implementation cp \
  --python-version 3.10 \
  --abi cp310

```
### 2. 执行最终总打包
```bash
cd /home/ld/OpenPLC
tar -czf openplc-v4.1.5-full-and-python310.tar.gz -C "$BUNDLE" .
```

## 2. 目标机离线部署 (RK3562)

将压缩包拷贝至目标机并解压。

~~~bash
# 1. 创建并进入安装目录
sudo mkdir -p /opt/openplc-offline
cd /opt/openplc-offline
sudo tar -xzf <你拷贝过来的压缩包名>

# 2. 将 Python 3.10 部署到目标位置
# 建议保持路径一致，或者根据你之前的习惯放在 /opt/python310
sudo cp -a python_env/python310 /opt/python310

# 3. 部署源码
sudo mkdir -p /opt/openplc-runtime
sudo tar -xzf src/openplc-runtime-v4.1.5.tar.gz -C /opt/openplc-runtime
cd /opt/openplc-runtime

~~~

### 2.1 设置环境变量
由于系统默认 Python 为 3.8，而我们需要链接到 `/root-ro/opt/python310`，必须在安装前注入以下变量：

```bash
export PYTHON_BIN=/root-ro/opt/python310/bin/python3.10
export SKIP_DEPS=1
export PIP_NO_INDEX=1
export PIP_FIND_LINKS=/root-ro/opt/openplc-offline/wheelhouse/
export PY310_PATH=/root-ro/opt/python310
export CFLAGS="-I$PY310_PATH/include/python3.10"
export LDFLAGS="-L$PY310_PATH/lib -lpython3.10"
```

### 2.2 核心配置文件修改 (已完成)
为了确保 C 核心能正确链接 Python 3.10 库，需确认以下修改：
- **CMakeLists.txt**: 设置 `RPATH` 指向 `$PY310_PATH/lib`。

编译 EtherCat , cmake 版本 要求 3.28 , 目标机只有 3.16 需要修改文件 才可以编译.
core/src/drivers/plugins/native/ethercat/libs/soem/CMakeLists.txt
```bash
cmake_minimum_required(VERSION 3.28)  # 根据需要修改指定版本
```

## 3. 安装与编译

运行安装脚本，使用 `-E` 传递上述环境变量。

```bash
# 执行安装 (会自动配置虚拟环境并编译 C 核心)
sudo -E ./install.sh
```

## 4. 运行与验证

### 4.1 修改 自动生成的 系统服务
```bash
cat /etc/systemd/system/openplc-runtime.service
[Unit]
Description=OpenPLC Runtime v4 Service
After=network.target

[Service]
Environment="PYTHONHOME=/opt/python310"
Environment="LD_LIBRARY_PATH=/opt/python310/lib:${LD_LIBRARY_PATH:-}"
Type=simple
Restart=always
RestartSec=5
User=root
Group=root
WorkingDirectory=/opt/openplc-runtime
ExecStart=/opt/openplc-runtime/start_openplc.sh

[Install]
WantedBy=multi-user.target

sudo systemctl daemon-reload
sudo systemctl restart openplc-runtime

```

### 4.2 验证步骤
1. **Web UI**: 访问 `https://<IP>:8443` (默认账号: `openplc` / `openplc`)。
2. **程序编译**: 在 Web 界面上传 `.st` 或 `.zip` 程序，点击 "Compile"。
   - 成功后会在 `build/` 目录下生成 `libplc_*.so`。
3. **日志检查**:
   - 观察控制台输出，确认 Python 插件（如 Modbus, OPCUA）加载成功。
   - 确认 EtherCAT 驱动（如使用）识别到从站数量。

---
*注：对于 Read-Only 根分区，请确保 `/etc/systemd/system/` 具备写入权限，或手动维护服务文件。*

