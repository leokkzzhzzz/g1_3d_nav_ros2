# 静态栅格地图导航配置

## 快速开始

### 1. 准备地图文件

**建图方式：使用FAST-LIVO2**

建图在独立的FAST-LIVO2工程中完成，最终得到：
- `.pcd` 点云文件（高质量3D地图）
- `.pgm` 栅格图像（2D导航地图）
- `.yaml` 地图元数据

将建好的地图文件复制到BotBrain工程：
```bash
# 复制地图文件到对应机器人的maps目录
cp your_map.pgm src/g1_pkg/maps/
cp your_map.yaml src/g1_pkg/maps/

# 可选：复制点云文件用于可视化
cp your_map.pcd src/g1_pkg/maps/
```

### 2. 配置默认地图
编辑 `robot_config.yaml`：
```yaml
robot_configuration:
  robot_name: "g1_robot"
  robot_model: "g1"
  default_static_map: "your_map.yaml"  # 添加这一行，指向你的地图文件
```

### 3. 启动导航系统
```bash
# 终端1：启动机器人基础服务
ros2 launch bot_bringup bringup.launch.py

# 终端2：启动静态地图定位
ros2 launch bot_localization static_map_localization.launch.py

# 终端3：启动导航
ros2 launch bot_navigation navigation.launch.py
```

## 目录结构

### 地图文件存放位置
```
botbrain_ws/
├── robot_config.yaml                     # 机器人配置（设置默认地图）
│
└── src/
    ├── bot_localization/
    │   ├── STATIC_MAP_README.md          # 本文件（快速入门）
    │   ├── STATIC_MAP_GUIDE.md           # 详细使用指南
    │   │
    │   └── bot_localization/
    │       ├── launch/
    │       │   └── static_map_localization.launch.py  # 静态地图定位启动
    │       │
    │       └── config/
    │           └── amcl_params.yaml      # AMCL定位参数
    │
    └── {robot_model}_pkg/                # 例如：g1_pkg, go2_pkg
        └── maps/                         # 地图存放目录
            ├── office_map.yaml           # 办公室地图元数据
            ├── office_map.pgm            # 办公室地图图像
            ├── office_map.pcd            # 办公室点云（可选）
            ├── warehouse_map.yaml        # 仓库地图元数据
            ├── warehouse_map.pgm         # 仓库地图图像
            └── warehouse_map.pcd         # 仓库点云（可选）
```

## 常用命令

### 地图管理
```bash
# 查看可用地图
ls -la src/g1_pkg/maps/*.yaml

# 复制新地图到maps目录
cp /path/to/new_map.pgm src/g1_pkg/maps/
cp /path/to/new_map.yaml src/g1_pkg/maps/
```

### 启动系统
```bash
# 1. 启动机器人基础服务
ros2 launch bot_bringup bringup.launch.py

# 2. 启动静态地图定位（使用默认地图）
ros2 launch bot_localization static_map_localization.launch.py

# 或指定特定地图文件
ros2 launch bot_localization static_map_localization.launch.py \
  map:=$(pwd)/src/g1_pkg/maps/office_map.yaml

# 3. 启动导航
ros2 launch bot_navigation navigation.launch.py
```

### 运行时切换地图
```bash
# 切换到不同的地图（无需重启）
ros2 service call /g1_robot/map_server/load_map \
  nav2_msgs/srv/LoadMap \
  "{map_url: '$(pwd)/src/g1_pkg/maps/warehouse_map.yaml'}"

# 注意：切换地图后需要在RViz中重新设置初始位姿
```

### 设置初始位姿
```bash
# 在RViz中使用 "2D Pose Estimate" 工具
# 或通过命令行
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

## 地图文件格式

### .yaml 元数据文件
FAST-LIVO2建图后生成的.yaml文件示例：
```yaml
image: office_map.pgm        # 对应的图像文件名
resolution: 0.05             # 分辨率：5cm/像素（FAST-LIVO2通常使用0.05-0.1）
origin: [-10.0, -10.0, 0.0]  # 地图原点坐标 [x, y, yaw]
negate: 0                    # 颜色反转：0=不反转
occupied_thresh: 0.65        # 占据阈值：>0.65视为障碍物
free_thresh: 0.196           # 空闲阈值：<0.196视为可通行
```

### .pgm 图像文件
- 格式：灰度图像（8位）
- 白色(255)：空闲空间（可通行）
- 黑色(0)：障碍物（不可通行）
- 灰色(127)：未知区域

### .pcd 点云文件（可选）
- 用于3D可视化
- 可在RViz中显示完整3D环境
- 不参与导航，仅用于参考

## 故障排查

### 问题1：找不到地图文件
```bash
# 检查地图文件是否存在
ls -la src/g1_pkg/maps/

# 检查robot_config.yaml配置
cat robot_config.yaml | grep default_static_map

# 确认.yaml文件中的image路径正确
cat src/g1_pkg/maps/your_map.yaml
```

### 问题2：定位不准确或丢失
```bash
# 方法1：在RViz中手动设置初始位姿
# 使用 "2D Pose Estimate" 工具点击机器人实际位置

# 方法2：调整AMCL参数（编辑 amcl_params.yaml）
max_particles: 5000        # 增加粒子数量
min_particles: 1000
update_min_d: 0.1          # 减小更新阈值
update_min_a: 0.1
```

### 问题3：地图显示异常
```bash
# 检查地图话题
ros2 topic echo /g1_robot/map --once

# 检查地图分辨率和大小
# 查看.yaml文件中的resolution参数
# FAST-LIVO2建图通常使用0.05-0.1米分辨率
```

### 问题4：激光扫描话题配置
```bash
# 查看可用的scan话题
ros2 topic list | grep scan

# 如果使用深度相机转激光，话题可能是：
# /g1_robot/front_camera/scan
# /g1_robot/scan

# 修改static_map_localization.launch.py中的scan_topic参数
```

### 问题5：切换地图后定位失败
```bash
# 切换地图后必须重新设置初始位姿
# 1. 在RViz中使用 "2D Pose Estimate"
# 2. 点击机器人在新地图中的实际位置
# 3. 等待AMCL粒子收敛
```

## AMCL参数优化

### 针对FAST-LIVO2高质量地图的推荐配置

FAST-LIVO2建图质量高，可以使用更激进的AMCL参数：

```yaml
# src/g1_pkg/config/amcl_params.yaml（如果存在）
# 或 src/bot_localization/bot_localization/config/amcl_params.yaml

amcl:
  ros__parameters:
    # 粒子数量（高质量地图可以用更少粒子）
    max_particles: 2000
    min_particles: 500
    
    # 更新阈值（高质量地图可以更频繁更新）
    update_min_d: 0.15        # 移动15cm更新
    update_min_a: 0.15        # 旋转0.15rad更新
    
    # 激光模型（高质量地图用likelihood_field更好）
    laser_model_type: "likelihood_field"
    laser_likelihood_max_dist: 2.0
    
    # 运动模型（根据机器人类型调整）
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    
    # 重采样
    resample_interval: 1      # 每帧重采样
```

### 不同场景的参数调整

**场景1：开阔空间（特征少）**
```yaml
max_particles: 5000          # 增加粒子数
laser_max_range: 10.0        # 增加激光范围
```

**场景2：狭窄空间（特征多）**
```yaml
max_particles: 1000          # 减少粒子数
update_min_d: 0.1            # 更频繁更新
```

**场景3：快速运动**
```yaml
transform_tolerance: 2.0     # 增加TF容差
update_min_d: 0.2            # 减少更新频率
```

## FAST-LIVO2建图 vs 原工程RTAB-Map

### 建图质量对比

| 特性 | 原工程(G1 RTAB-Map) | FAST-LIVO2建图 |
|------|-------------------|---------------|
| 传感器融合 | ❌ 仅LiDAR | ✅ LiDAR+相机+IMU |
| 视觉信息 | ❌ 无 | ✅ 有 |
| IMU辅助 | ❌ 无 | ✅ 紧耦合 |
| 去畸变 | ⚠️ 简单 | ✅ 精确 |
| 建图精度 | ⚠️ 中等 | ✅ 高 |
| 闭环检测 | ⚠️ 仅几何 | ✅ 几何+视觉 |
| 动态环境 | ⚠️ 一般 | ✅ 更好 |

### 导航性能对比

| 指标 | RTAB-Map定位 | AMCL定位(FAST-LIVO2地图) |
|------|-------------|------------------------|
| CPU使用 | 20-40% | 5-15% |
| 内存使用 | 500MB-8GB | 10MB-200MB |
| 定位频率 | 1-5 Hz | 10-20 Hz |
| 初始化 | 5-10秒 | 1-2秒 |
| 地图质量 | 中等 | 高（FAST-LIVO2） |

### 推荐方案

**FAST-LIVO2建图 + AMCL定位 = 最佳组合**
- ✅ 建图质量最高（多传感器融合）
- ✅ 导航资源消耗低（AMCL轻量）
- ✅ 地图可编辑优化
- ✅ 支持多地图切换

## 多地图管理

### 场景1：多楼层建筑

```bash
# 为每层建立独立地图
src/g1_pkg/maps/
├── floor_1.yaml
├── floor_1.pgm
├── floor_2.yaml
├── floor_2.pgm
├── floor_3.yaml
└── floor_3.pgm

# 切换楼层时切换地图
ros2 service call /g1_robot/map_server/load_map \
  nav2_msgs/srv/LoadMap \
  "{map_url: '$(pwd)/src/g1_pkg/maps/floor_2.yaml'}"
```

### 场景2：室内外切换

```bash
# 室内和室外使用不同地图
src/g1_pkg/maps/
├── indoor.yaml
├── indoor.pgm
├── outdoor.yaml
└── outdoor.pgm

# 进入室外时切换
ros2 service call /g1_robot/map_server/load_map \
  nav2_msgs/srv/LoadMap \
  "{map_url: '$(pwd)/src/g1_pkg/maps/outdoor.yaml'}"
```

### 场景3：多区域仓库

```bash
# 不同区域使用不同地图
src/g1_pkg/maps/
├── warehouse_a.yaml
├── warehouse_a.pgm
├── warehouse_b.yaml
└── warehouse_b.pgm
```

## 参考资源

### 导航相关
- [Nav2 Documentation](https://navigation.ros.org/)
- [AMCL Configuration](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [Map Server](https://navigation.ros.org/configuration/packages/configuring-map-server.html)

### FAST-LIVO2相关
- [FAST-LIVO2 GitHub](https://github.com/hku-mars/FAST-LIVO2)
- FAST-LIVO2论文：多传感器紧耦合LiDAR-Inertial-Visual里程计

### 地图处理工具
- [PCL (Point Cloud Library)](https://pointclouds.org/)
- [Open3D](http://www.open3d.org/)
- [Octomap](https://octomap.github.io/)

## 工作流程总结

### 完整流程

```
1. FAST-LIVO2建图（在独立工程）
   ├─ 运行FAST-LIVO2
   ├─ 得到高质量点云(.pcd)
   └─ 转换为2D栅格地图(.pgm + .yaml)

2. 复制地图到BotBrain
   └─ cp *.pgm *.yaml src/g1_pkg/maps/

3. 配置BotBrain
   └─ 编辑robot_config.yaml设置default_static_map

4. 启动导航系统
   ├─ ros2 launch bot_bringup bringup.launch.py
   ├─ ros2 launch bot_localization static_map_localization.launch.py
   └─ ros2 launch bot_navigation navigation.launch.py

5. 设置初始位姿
   └─ 在RViz中使用"2D Pose Estimate"

6. 开始导航
   └─ 在RViz中使用"2D Nav Goal"发送目标
```

## 下一步

1. ✅ 在FAST-LIVO2工程中建立地图
2. ✅ 将地图文件复制到BotBrain
3. ✅ 配置robot_config.yaml
4. ✅ 启动导航系统测试
5. 📚 阅读STATIC_MAP_GUIDE.md了解详细配置
6. 🎯 根据实际情况优化AMCL参数
7. 🚀 部署到实际应用

祝你使用愉快！🤖
