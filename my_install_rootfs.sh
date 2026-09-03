I'll inspect `install.sh` and the project setup to provide the most direct and reliable steps for installing OpenPLC inside the chroot environment. 


I'll explain how to directly and quickly deploy OpenPLC Runtime into the mounted `rootfs.img` using QEMU chroot.在 `chroot` 环境中（配有 `qemu-aarch64-static`），**QEMU 会直接仿真 ARM64 指令集的执行**，这意味着你**不需要在宿主机上打打包、也不需要复杂的离线 `pip download` 参数**。你此时就相当于直接站在了目标机系统内部！

最简单、快速的部署流程如下：

---

### 第一步：在宿主机准备挂载与代码复制（Host 终端）

在宿主机（未进入 `chroot` 状态）执行以下命令，把宿主机网络配置和源码直接拷进 `rootfs`：

```bash
# 1. 复制宿主机的 DNS 配置到 rootfs，确保 chroot 内部可以正常联网下载 pip 包 (可选)
sudo cp /etc/resolv.conf ~/mount_dir/etc/resolv.conf

# 2. 挂载必要的伪文件系统（防止编译/安装时缺少 /proc 或 /dev 报异常）
sudo mount -t proc /proc ~/mount_dir/proc
sudo mount -t sysfs /sys ~/mount_dir/sys
sudo mount --bind /dev ~/mount_dir/dev
sudo mount --bind /dev/pts ~/mount_dir/dev/pts

# 3. 直接将宿主机的 OpenPLC 源码复制到 rootfs 的 /opt/openplc-runtime
sudo mkdir -p ~/mount_dir/opt/openplc-runtime
sudo cp -a /home/ld/OpenPLC/openplc-runtime/. ~/mount_dir/opt/openplc-runtime/
sudo rsync -av  --exclude='.vscode'  --exclude='.github'   --exclude='.claude'   /home/ld/OpenPLC/openplc-runtime/ ~/mount_dir/opt/openplc-runtime/
```

---

### 第二步：进入 chroot 环境（Host 终端）

```bash
sudo mount -o loop rootfs.img ~/mount_dir 
sudo chroot ~/mount_dir
```
安装 cmake >= 3.28
# 在 chroot 内部运行
pip install -i https://pypi.tuna.tsinghua.edu.cn/simple cmake
---

### 第三步：在 chroot 中一键编译与安装（Chroot 终端）

进入 `chroot` 后，由于挂载了网络，可以直接用系统自带的 Python 3.10 进行联网安装和编译：

```bash
cd /opt/openplc-runtime

# 1. 设置环境变量，指向 rootfs 中的 Python 3.10
export PYTHON_BIN=/opt/python310/bin/python3.10
export PY310_PATH=/opt/python310
export CFLAGS="-I$PY310_PATH/include/python3.10"
export LDFLAGS="-L$PY310_PATH/lib -lpython3.10"

# 2. 升级基础工具包 (可选,建议升级)：
"$PYTHON_BIN" -m pip install --upgrade pip setuptools wheel

# 3. 执行脚本一键完成 C 核心编译与 Python 虚拟环境配置
./install.sh
```

---

### 第四步：配置开机自启 Service（Chroot 终端）

在 `chroot` 中写入开机自启服务文件：

```bash
cat << 'EOF' > /etc/systemd/system/openplc-runtime.service
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
EOF

# 在 multi-user.target.wants 中创建软链接，完成开机使能
systemctl enable openplc-runtime.service #( 推荐)
# 或者 手动处理:
mkdir -p /etc/systemd/system/multi-user.target.wants
ln -sf /etc/systemd/system/openplc-runtime.service /etc/systemd/system/multi-user.target.wants/openplc-runtime.service
```

---

### 第五步：退出并解除挂载（Host 终端）

在 `chroot` 终端中输入 `exit` 返回宿主机，清理挂载目录：

```bash
# 1. 退出 chroot
exit

# 2. 在宿主机卸载伪文件系统
sudo umount ~/mount_dir/dev/pts
sudo umount ~/mount_dir/dev
sudo umount ~/mount_dir/sys
sudo umount ~/mount_dir/proc

# 3. 卸载镜像
sudo umount ~/mount_dir
```

---

### 为什么这个方案最快最省事？

1. **不需要打包/解压压缩包**：宿主机直接 `cp -a` 目录到挂载点 `~/mount_dir/opt/`，秒级传输。
2. **无需处理交叉编译/下载平台参数**：因为配合 `qemu-aarch64-static`，`pip install` 会自动识别出当前是 ARM64 + CPython 3.10 环境并自动下载正确的 wheel 包。
3. **一次性固化到镜像**：`install.sh` 编译出的可执行文件 `build/plc_main`、生成的插件环境以及开机服务配置，都已经直接写入到了 `rootfs.img` 中。烧录镜像到 RK3562 后，通电开机即可直接运行。