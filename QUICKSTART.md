# 🚀 快速开始指南

## 项目已整理完成 ✅

项目结构已经重新整理，更加清晰易用！

---

## 📁 新的项目结构

```
AprilTag_ROS2_intel-D435/
├── README.md              # 完整文档
├── QUICKSTART.md          # 本文件 - 快速开始
├── STRUCTURE.md           # 详细结构说明
├── ros2_ws/              # ROS 2 工作空间 ⭐
│   └── src/              # 所有 ROS 2 包
├── docs/                 # 所有文档
├── docker_config/        # Docker 配置
├── images/               # 图片资源
├── config/               # 配置文件
├── scripts/              # 脚本工具
├── camera_info/          # 相机标定
└── data/                 # 生成数据
```

---

## ⚡ 快速使用（3 步骤）

### 1️⃣ 构建 ROS 2 工作空间

```bash
cd ~/AprilTag_ROS2_intel-D435/ros2_ws
colcon build --symlink-install
```

### 2️⃣ 激活环境

```bash
source ~/AprilTag_ROS2_intel-D435/ros2_ws/install/setup.bash
```

**提示**: 将这行加入 `~/.bashrc` 以自动激活：
```bash
echo "source ~/AprilTag_ROS2_intel-D435/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 3️⃣ 运行程序

```bash
# 多标签地图跟踪
ros2 run apriltag_detector apriltag_map

# 位置验证
ros2 run apriltag_detector camera_validator

# 手眼标定数据采集
ros2 run apriltag_detector record_calibration
```

---

## 🔧 常用命令

### 重新构建

```bash
cd ~/AprilTag_ROS2_intel-D435/ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

### 检查包是否安装

```bash
ros2 pkg list | grep apriltag
```

应该显示：
```
apriltag_detector
apriltag_msgs
```

### 查看可用命令

```bash
ros2 run apriltag_detector <TAB><TAB>
```

---

## 📚 详细文档

- **完整说明**: 查看 `README.md`
- **项目结构**: 查看 `STRUCTURE.md`
- **安装指南**: 查看 `docs/SETUP_GUIDE.md`
- **ROS 2 设置**: 查看 `docs/ROS2_SETUP.md`
- **Docker 部署**: 查看 `docs/DOCKER_SETUP.md`

---

## 🐳 使用 Docker（可选）

```bash
cd ~/AprilTag_ROS2_intel-D435/docker_config
docker-compose up
```

---

## ❓ 故障排除

### 问题: `Package 'apriltag_detector' not found`

**解决方案**:
```bash
cd ~/AprilTag_ROS2_intel-D435/ros2_ws
source install/setup.bash
```

### 问题: 构建失败

**解决方案**:
```bash
cd ~/AprilTag_ROS2_intel-D435/ros2_ws
rm -rf build install log
colcon build --symlink-install
```

### 问题: 找不到相机

**解决方案**:
```bash
# 检查 RealSense 相机
rs-enumerate-devices

# 重新插拔 USB 连接
```

---

## 📊 整理改进

### ✨ 改进项

- ✅ 清晰的 ROS 2 工作空间结构 (`ros2_ws/`)
- ✅ 文档集中管理 (`docs/`)
- ✅ 配置分离 (`config/`, `docker_config/`)
- ✅ 资源整理 (`images/`, `scripts/`)
- ✅ 添加 `.gitignore` 忽略构建产物
- ✅ 标准的 ROS 2 工作空间布局

### 📈 使用对比

**整理前**:
```bash
cd ~/AprilTag_ROS2_intel-D435
colcon build --base-paths apriltag_detector
source install/setup.bash
ros2 run apriltag_detector apriltag_map
```

**整理后**:
```bash
cd ~/AprilTag_ROS2_intel-D435/ros2_ws
colcon build
source install/setup.bash
ros2 run apriltag_detector apriltag_map
```

更加清晰和标准！

---

## 🎯 下一步

1. 阅读完整的 `README.md`
2. 查看 `STRUCTURE.md` 了解项目结构
3. 根据需求运行相应程序
4. 查看 `docs/` 获取详细文档

---

**整理完成日期**: 2025年11月25日  
**状态**: ✅ 可以正常使用

如有问题，请查看 `README.md` 的故障排除部分。
