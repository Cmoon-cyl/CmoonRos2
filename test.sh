#!/bin/bash
set -e

if [ -z "$SUDO_USER" ]; then
    user_home="$HOME"  # 如果没有使用 sudo，使用当前用户的 HOME
else
    user_home="/home/$SUDO_USER"  # 如果使用了 sudo，获取原始用户的 HOME
fi

echo "userhome:$user_home"
# 定义路径
local_dir="$user_home/CmoonRos3"
container_dir="/home/cmoon/cmoon_ws"
docker_image="cmooncyl/ustc1010:cmoonros2"
script_path="$user_home/.cmoon/bin/cmoonros3"


# install nvidia docker
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

echo "Adding the NVIDIA container toolkit repository..."
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# Update the package list
echo "Updating the apt package list..."
sudo apt update

# Install the NVIDIA container toolkit
echo "Installing the NVIDIA container toolkit..."
sudo apt-get install -y nvidia-container-toolkit

# Configure the NVIDIA Container Toolkit
echo "Configuring the NVIDIA Container Toolkit..."
sudo nvidia-ctk runtime configure --runtime=docker

# Restart the Docker service
echo "Restarting the Docker service..."
sudo systemctl restart docker

echo "NVIDIA container toolkit installation and configuration complete."

# 拉取 Docker 镜像
if ! docker pull $docker_image; then
    echo "Error: Failed to pull Docker image $docker_image."
    exit 1
fi

# 创建本地目录
if mkdir -p $local_dir; then
    echo "目录 $local_dir 创建成功。"
    [ ! -z "$SUDO_USER" ] && sudo chown -R $SUDO_USER:$SUDO_USER "$local_dir"
else
    echo "错误：无法创建目录 $local_dir。"
    exit 1
fi

# 在后台运行 Docker 容器，并获取容器 ID
container_id=$(docker run -dit --name cmoonros2_tmp $docker_image)
echo "container_id: $container_id"

# 检查容器是否成功启动
if [ -z "$container_id" ]; then
    echo "Error: Failed to start the Docker container."
    exit 1
fi

# 从容器中拷贝 cmoon_ws 文件夹到本地目录
if ! docker cp "$container_id:$container_dir" "$local_dir"; then
    echo "Error: Failed to copy files from the Docker container."
    docker rm -f $container_id
    exit 1
else
    echo "cmoon_ws 文件夹已拷贝到 $local_dir。"
    [ ! -z "$SUDO_USER" ] && sudo chown -R $SUDO_USER:$SUDO_USER "$local_dir"
fi

# 删除临时容器
docker rm -f $container_id

echo "cmoon_ws folder copied to $local_dir."

# 创建新容器并挂载 cmoon_ws 文件夹
if ! docker run --ipc=host --gpus all -dit --name=cmoonros3 --privileged \
-v /dev:/dev \
-v $local_dir:/home/cmoon:Z \
-v /tmp/.X11-unix:/tmp/.X11-unix \
-e DISPLAY=unix$DISPLAY \
-w /home/cmoon \
--net=host \
$docker_image; then
    echo "Error: Failed to run the new Docker container."
    exit 1
fi

# 创建目录用于存放脚本
mkdir -p $(dirname "$script_path")

# 写入脚本
cat << 'EOF' > "$script_path"
xhost +local: >> /dev/null
echo "请输入指令控制humble: 重启(r) 进入(e) 启动(s) 关闭(c) 删除(d) 测试(t):"
read choose
case $choose in
    s) docker start cmoonros3;;
    r) docker restart cmoonros3;;
    e) docker exec -it cmoonros3 /bin/bash;;
    c) docker stop cmoonros3;;
    d) docker stop cmoonros3 && docker rm cmoonros3;;
    t) docker exec -it cmoonros3 /bin/bash -c "source /ros_entrypoint.sh && ros2";;
esac
newgrp docker
EOF

# 更改权限使脚本可执行
[ ! -z "$SUDO_USER" ] && sudo chown -R $SUDO_USER:$SUDO_USER "$user_home/.cmoon"
chmod +x "$script_path"

# 将新的 bin 目录添加到 PATH
if ! grep -q "export PATH=\$PATH:$user_home/.cmoon/bin/" $user_home/.zshrc; then
    echo "export PATH=\$PATH:$user_home/.cmoon/bin/" >> $user_home/.zshrc
fi

# 向 .bashrc 添加 source 命令
source_command="source $local_dir/cmoon_ws/install/setup.bash"
if ! grep -q "$source_command" $user_home/.bashrc; then
    echo "$source_command" >> $user_home/.bashrc
fi

echo "Install Complete."