# 使用FAST-LIVO2地图进行导航 - 详细指南

## 概述

本指南说明如何使用FAST-LIVO2建立的高质量地图在BotBrain系统中进行定位和导航。

### 系统架构

```
┌─────────────────────────────────────────────────────────┐
│              FAST-LIVO2建图 + AMCL导航方案                │
├─────────────────────────────────────────────────────────┤
│                                                           │
│  建图阶段（FAST-LIVO2工程）                                │
│  ┌──────────────────────────────────────────────┐       │
│  │ LiDAR + 相机 + IMU → FAST-LIVO2 → .pcd点云   │       │
│  │                                    ↓          │       │
│  │                            转换为.pgm/.yaml   │       │
│  └──────────────────────────────────────────────┘       │
│                        ↓                                  │
│  导航阶段（BotBrain工程）                                 │
│  ┌──────────────────────────────────────────────┐       │
│  │ .pgm/.yaml → map_server → AMCL → Nav2       │       │
│  │              (加载地图)   (定位)  (导航)      │       │
│  └──────────────────────────────────────────────┘       │
│                                                           │
└─────────────────────────────────────────────────────────┘
```

### 方案优势

**FAST-LIVO2建图：**
- ✅ 多传感器紧耦合融合（LiDAR + 相机 + IMU）
- ✅ 高精度建图（IMU辅助去畸变）
- ✅ 视觉+几何双重闭环检测
- ✅ 全局一致性好
- ✅ 适应动态环境

**AMCL定位：**
- ✅ 资源消耗低（CPU 5-15%，内存 10-200MB）
- ✅ 定位频率高（10-20Hz）
- ✅ 成熟稳定
- ✅ 配置简单

**组合效果：**
- 🎯 建图质量高 + 导航资源消耗低
- 🎯 最佳性价比方案

## 目录结构

```
botbrain_ws/
├── robot_config.yaml                          # 机器人配置
│
└── src/
    ├── bot_localization/
    │   ├── STATIC_MAP_README.md               # 快速入门
    │   ├── STATIC_MAP_GUIDE.md                # 本文件（详细指南）
    │   │
    │   └── bot_localization/
    │       ├── launch/
    │       │   └── static_map_localization.launch.py  # 定位启动文件
    │       │
    │       └── config/
    │           └── amcl_params.yaml           # AMCL参数配置
    │
    ├── bot_navigation/
    │   └── launch/
    │       └── navigation.launch.py           # 导航启动文件
    │
    └── {robot_model}_pkg/                     # 例如：g1_pkg
        ├── maps/                              # 地图存放目录
        │   ├── office_map.yaml                # 办公室地图元数据
        │   ├── office_map.pgm                 # 办公室地图图像
        │   ├── office_map.pcd                 # 办公室点云（可选）
        │   ├── warehouse_map.yaml             # 仓库地图
        │   └── warehouse_map.pgm
        │
        └── config/
            ├── nav2_params.yaml               # Nav2导航参数
            └── amcl_params.yaml               # 机器人专用AMCL参数（可选）
```

## 地图文件格式

### .yaml 元数据文件

FAST-LIVO2建图后生成的.yaml文件格式：

```yaml
image: office_map.pgm           # 地图图像文件名（相对路径）
resolution: 0.05                # 分辨率（米/像素）
                                # FAST-LIVO2通常使用0.05-0.1米
origin: [-10.0, -10.0, 0.0]    # 地图原点（x, y, yaw）
                                # 地图左下角在世界坐标系中的位置
negate: 0                       # 是否反转颜色（0=否，1=是）
occupied_thresh: 0.65           # 占据阈值（>此值为障碍物）
free_thresh: 0.196              # 空闲阈值（<此值为可通行）
```

**参数说明：**
- `resolution`: FAST-LIVO2建图精度高，推荐0.05米（5cm）
- `origin`: 地图坐标系原点，通常是建图起始点
- `occupied_thresh`: 灰度值>0.65*255=166视为障碍物
- `free_thresh`: 灰度值<0.196*255=50视为可通行区域

### .pgm 图像文件

- **格式**: 灰度图像（8位，0-255）
- **白色(255)**: 空闲空间（可通行）
- **黑色(0)**: 障碍物（不可通行）
- **灰色(127)**: 未知区域
- **文件大小**: 取决于地图范围和分辨率
  - 100m×100m @ 0.05m分辨率 ≈ 4MB
  - 200m×200m @ 0.05m分辨率 ≈ 16MB

### .pcd 点云文件（可选）

- **用途**: 3D可视化，不参与导航
- **格式**: ASCII或Binary PCD格式
- **内容**: 3D点云数据（x, y, z坐标）
- **使用**: 可在RViz中显示完整3D环境

## 配置步骤

### 1. 准备地图文件

从FAST-LIVO2工程获取地图文件：

```bash
# 假设FAST-LIVO2建图后得到以下文件：
# - office_map.pcd   (点云文件)
# - office_map.pgm   (2D栅格地图)
# - office_map.yaml  (地图元数据)

# 复制到BotBrain工程
cd /path/to/botbrain_ws
cp /path/to/fast-livo2/maps/office_map.pgm src/g1_pkg/maps/
cp /path/to/fast-livo2/maps/office_map.yaml src/g1_pkg/maps/

# 可选：复制点云用于可视化
cp /path/to/fast-livo2/maps/office_map.pcd src/g1_pkg/maps/
```

**注意事项：**
- 确保.yaml文件中的`image`字段指向正确的.pgm文件名
- 如果.yaml和.pgm不在同一目录，需要修改`image`字段为相对路径
- 检查地图文件权限，确保可读

### 2. 验证地图文件

```bash
# 检查文件是否存在
ls -lh src/g1_pkg/maps/office_map.*

# 查看.yaml内容
cat src/g1_pkg/maps/office_map.yaml

# 使用图像查看器检查.pgm
# Linux:
eog src/g1_pkg/maps/office_map.pgm
# 或
gimp src/g1_pkg/maps/office_map.pgm
```

### 3. 配置默认地图

编辑 `botbrain_ws/robot_config.yaml`：

```yaml
robot_configuration:
  robot_name: "g1_robot"
  robot_model: "g1"
  description_file_type: "xacro"
  network_interface: "eth0"
  
  # 添加静态地图配置
  default_static_map: "office_map.yaml"  # 默认加载的地图文件名
```

### 4. 配置激光雷达话题（如果需要）

检查你的激光雷达话题名称：

```bash
# 启动机器人后查看话题
ros2 topic list | grep scan

# 常见话题名称：
# /g1_robot/scan
# /g1_robot/front_camera/scan  (如果使用depth_to_laserscan)
# /scan
```

如果话题名称不是`/scan`，需要修改启动文件：

编辑 `src/bot_localization/bot_localization/launch/static_map_localization.launch.py`：

```python
# 找到AMCL节点配置，修改scan_topic
parameters=[amcl_params_file, {
    'use_sim_time': use_sim_time,
    'base_frame_id': f'{prefix}base_link',
    'odom_frame_id': f'{prefix}odom',
    'global_frame_id': f'{prefix}map',
    'scan_topic': f'front_camera/scan',  # 修改为你的话题名
}],
```

## 使用方法

### 完整启动流程

```bash
# ========== 步骤1：启动机器人基础服务 ==========
# 终端1
cd /path/to/botbrain_ws
source install/setup.bash
ros2 launch bot_bringup bringup.launch.py

# 等待机器人初始化完成...

# ========== 步骤2：启动静态地图定位 ==========
# 终端2
cd /path/to/botbrain_ws
source install/setup.bash
ros2 launch bot_localization static_map_localization.launch.py

# 系统会自动加载robot_config.yaml中配置的default_static_map

# ========== 步骤3：启动导航 ==========
# 终端3
cd /path/to/botbrain_ws
source install/setup.bash
ros2 launch bot_navigation navigation.launch.py

# ========== 步骤4：设置初始位姿 ==========
# 在RViz中：
# 1. 点击工具栏的 "2D Pose Estimate"
# 2. 在地图上点击机器人的实际位置
# 3. 拖动鼠标设置机器人朝向
# 4. 观察AMCL粒子云收敛

# ========== 步骤5：发送导航目标 ==========
# 在RViz中：
# 1. 点击工具栏的 "2D Nav Goal"
# 2. 在地图上点击目标位置
# 3. 拖动鼠标设置目标朝向
# 4. 机器人开始自主导航
```

### 指定特定地图启动

```bash
# 使用特定地图文件（覆盖默认配置）
ros2 launch bot_localization static_map_localization.launch.py \
  map:=$(pwd)/src/g1_pkg/maps/warehouse_map.yaml

# 使用绝对路径
ros2 launch bot_localization static_map_localization.launch.py \
  map:=/home/user/maps/custom_map.yaml
```

## FAST-LIVO2地图质量优化

### 从FAST-LIVO2获取最佳地图

**建图建议：**
1. **移动速度**: 保持匀速慢速移动（0.3-0.5 m/s）
2. **覆盖范围**: 完整覆盖所有需要导航的区域
3. **闭环路径**: 多次经过同一区域以触发闭环
4. **光照条件**: 保持稳定光照（相机需要）
5. **避免动态物体**: 建图时尽量避开移动的人和物体

**点云转2D地图参数：**
```python
# 如果你需要自己转换.pcd到.pgm
# 推荐参数：
resolution = 0.05          # 5cm分辨率（高精度）
height_range = (0.1, 2.0)  # 考虑0.1-2.0米高度的障碍物
robot_radius = 0.3         # 机器人半径，用于障碍物膨胀
```

### 地图后处理

如果需要优化地图质量，可以使用图像编辑工具：

```bash
# 使用GIMP编辑.pgm文件
gimp src/g1_pkg/maps/office_map.pgm

# 常见优化操作：
# 1. 清理噪点（滤波器 → 去噪）
# 2. 填补小空洞（选择 → 按颜色 → 填充）
# 3. 平滑边缘（滤波器 → 模糊 → 高斯模糊）
# 4. 手动标记禁行区域（画笔工具，黑色）
# 5. 保存为.pgm格式（导出 → 选择PGM）
```

## 多地图管理

### 组织多个地图

```bash
# 推荐的地图命名规范
src/g1_pkg/maps/
├── building_a_floor_1.yaml
├── building_a_floor_1.pgm
├── building_a_floor_2.yaml
├── building_a_floor_2.pgm
├── building_b_warehouse.yaml
├── building_b_warehouse.pgm
├── outdoor_parking.yaml
└── outdoor_parking.pgm
```

### 场景1：多楼层建筑

```bash
# 为每层建立独立地图
# 在FAST-LIVO2中分别建图

# 切换楼层时切换地图
ros2 service call /g1_robot/map_server/load_map \
  nav2_msgs/srv/LoadMap \
  "{map_url: '$(pwd)/src/g1_pkg/maps/building_a_floor_2.yaml'}"

# 重新设置初始位姿
# 在RViz中使用"2D Pose Estimate"
```

### 场景2：室内外切换

```bash
# 室内和室外使用不同地图
# FAST-LIVO2分别建图

# 进入室外时切换
ros2 service call /g1_robot/map_server/load_map \
  nav2_msgs/srv/LoadMap \
  "{map_url: '$(pwd)/src/g1_pkg/maps/outdoor_parking.yaml'}"
```

### 场景3：大型区域分割

对于超大区域，可以分割为多个地图：

```bash
# 分区建图
maps/
├── zone_a.yaml  # 区域A
├── zone_a.pgm
├── zone_b.yaml  # 区域B
├── zone_b.pgm
├── zone_c.yaml  # 区域C
└── zone_c.pgm

# 机器人在不同区域时切换对应地图
# 可以通过GPS、二维码等方式自动识别区域
```

### 自动地图切换（高级）

可以编写脚本实现自动地图切换：

```python
#!/usr/bin/env python3
"""
基于位置的自动地图切换
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.srv import LoadMap

class AutoMapSwitcher(Node):
    def __init__(self):
        super().__init__('auto_map_switcher')
        
        # 定义地图区域
        self.map_zones = {
            'floor_1': {
                'map': 'floor_1.yaml',
                'bounds': {'x': (0, 50), 'y': (0, 30)}
            },
            'floor_2': {
                'map': 'floor_2.yaml',
                'bounds': {'x': (0, 50), 'y': (30, 60)}
            }
        }
        
        self.current_map = None
        
        # 订阅机器人位置
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.pose_callback,
            10
        )
        
        # 创建地图切换客户端
        self.map_client = self.create_client(LoadMap, '/map_server/load_map')
    
    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        
        # 检查机器人在哪个区域
        for zone_name, zone_info in self.map_zones.items():
            bounds = zone_info['bounds']
            if (bounds['x'][0] <= x <= bounds['x'][1] and
                bounds['y'][0] <= y <= bounds['y'][1]):
                
                # 如果需要切换地图
                if self.current_map != zone_info['map']:
                    self.switch_map(zone_info['map'])
                    self.current_map = zone_info['map']
                break
    
    def switch_map(self, map_name):
        request = LoadMap.Request()
        request.map_url = f'/path/to/maps/{map_name}'
        self.map_client.call_async(request)
        self.get_logger().info(f'Switching to map: {map_name}')

def main():
    rclpy.init()
    node = AutoMapSwitcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 调试和优化

### 查看地图

```bash
# 在RViz中查看
rviz2

# 添加显示：
# - Add -> Map -> Topic: /map
# - Add -> LaserScan -> Topic: /scan
# - Add -> TF
# - Add -> PoseArray -> Topic: /particle_cloud (AMCL粒子)
```

### AMCL定位问题排查

**问题1：定位不准确**
```yaml
# 增加粒子数量
max_particles: 5000
min_particles: 1000

# 减小更新阈值
update_min_d: 0.1
update_min_a: 0.1
```

**问题2：定位丢失**
```yaml
# 增加恢复参数
recovery_alpha_fast: 0.1
recovery_alpha_slow: 0.001

# 放宽变换容差
transform_tolerance: 2.0
```

**问题3：初始定位困难**
```bash
# 在RViz中手动设置初始位姿
# 使用 "2D Pose Estimate" 工具

# 或通过服务设置
ros2 service call /g1_robot/amcl/set_initial_pose \
  geometry_msgs/srv/PoseWithCovarianceStamped \
  "{pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

### 地图质量优化

**提高地图质量：**
1. 建图时移动速度要慢
2. 确保激光雷达数据质量好
3. 覆盖所有需要导航的区域
4. 避免动态物体（人、车等）

**地图后处理：**
```bash
# 使用GIMP或其他图像编辑工具
# 1. 打开.pgm文件
# 2. 清理噪点
# 3. 填补空洞
# 4. 保存为.pgm格式
```

## 性能对比

### 内存使用

| 方案 | 小地图(100m²) | 中地图(500m²) | 大地图(2000m²) |
|------|--------------|--------------|---------------|
| RTAB-Map | ~500MB | ~2GB | ~8GB |
| 静态地图 | ~10MB | ~50MB | ~200MB |

### CPU使用

| 方案 | 定位CPU | 建图CPU |
|------|---------|---------|
| RTAB-Map | 20-40% | 40-80% |
| AMCL | 5-15% | N/A（离线建图） |

## 常见问题

**Q: 可以同时使用RTAB-Map和静态地图吗？**
A: 不建议。选择一种方案即可。如果需要切换，停止一个再启动另一个。

**Q: 静态地图可以更新吗？**
A: 不能实时更新。需要重新建图并替换地图文件。

**Q: 如何选择RTAB-Map还是静态地图？**
A: 
- 有RGB-D相机 → RTAB-Map
- 只有2D激光雷达 → 静态地图
- 环境经常变化 → RTAB-Map
- 环境固定 → 静态地图
- 需要3D信息 → RTAB-Map
- 只需2D导航 → 静态地图

**Q: 深度相机可以用于AMCL吗？**
A: 可以。使用depth_to_laserscan将深度图转换为2D激光扫描。

**Q: 地图分辨率如何选择？**
A: 
- 0.05m（5cm）：高精度，文件大
- 0.10m（10cm）：平衡，推荐
- 0.20m（20cm）：低精度，文件小

## 示例：完整工作流程

```bash
# ========== 第一步：建图 ==========
# 1. 启动机器人
ros2 launch bot_bringup bringup.launch.py

# 2. 启动SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py

# 3. 遥控建图
# 使用手柄控制机器人移动

# 4. 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/office_map

# 5. 复制地图
cp ~/office_map.* botbrain_ws/src/g1_pkg/maps/

# ========== 第二步：配置 ==========
# 6. 编辑robot_config.yaml
# default_static_map: "office_map.yaml"

# ========== 第三步：导航 ==========
# 7. 重启系统
ros2 launch bot_bringup bringup.launch.py

# 8. 启动静态地图定位
ros2 launch bot_localization static_map_localization.launch.py

# 9. 启动导航
ros2 launch bot_navigation navigation.launch.py

# 10. 在RViz中设置初始位姿和导航目标
```

## 参考资料

- [Nav2 Documentation](https://navigation.ros.org/)
- [AMCL Documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [Map Server Documentation](https://navigation.ros.org/configuration/packages/configuring-map-server.html)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Cartographer](https://google-cartographer-ros.readthedocs.io/)

## 技术支持

如有问题，请查看：
- ROS 2 Answers: https://answers.ros.org/
- Nav2 GitHub Issues: https://github.com/ros-planning/navigation2/issues


## AMCL参数配置

### 针对FAST-LIVO2高质量地图的优化配置

FAST-LIVO2建图质量高，可以使用更优化的AMCL参数。

编辑 `src/bot_localization/bot_localization/config/amcl_params.yaml`：

```yaml
amcl:
  ros__parameters:
    use_sim_time: False
    
    # ========== 粒子滤波参数 ==========
    # FAST-LIVO2地图质量高，可以用较少粒子
    max_particles: 2000              # 最大粒子数
    min_particles: 500               # 最小粒子数
    
    # ========== 更新阈值 ==========
    # 高质量地图可以更频繁更新
    update_min_d: 0.15               # 移动15cm触发更新
    update_min_a: 0.15               # 旋转0.15rad触发更新
    
    # ========== 坐标系配置 ==========
    base_frame_id: "base_link"
    odom_frame_id: "odom"
    global_frame_id: "map"
    
    # ========== 激光模型 ==========
    # likelihood_field适合高质量地图
    laser_model_type: "likelihood_field"
    laser_likelihood_max_dist: 2.0   # 激光匹配最大距离
    laser_max_range: 100.0           # 激光最大范围
    laser_min_range: -1.0            # 激光最小范围（-1=使用传感器默认）
    max_beams: 60                    # 使用的激光束数量
    
    # ========== 运动模型 ==========
    # 根据机器人类型选择
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    # 或 "nav2_amcl::OmniMotionModel" (全向移动)
    
    # 运动噪声参数（根据机器人调整）
    alpha1: 0.2                      # 旋转噪声（旋转部分）
    alpha2: 0.2                      # 旋转噪声（平移部分）
    alpha3: 0.2                      # 平移噪声（平移部分）
    alpha4: 0.2                      # 平移噪声（旋转部分）
    alpha5: 0.2                      # 平移噪声（侧向）
    
    # ========== 重采样 ==========
    resample_interval: 1             # 每帧重采样
    
    # ========== TF发布 ==========
    tf_broadcast: true               # 发布map→odom变换
    transform_tolerance: 1.0         # TF容差（秒）
    
    # ========== 初始位姿 ==========
    set_initial_pose: false          # 是否使用参数设置初始位姿
    always_reset_initial_pose: false
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
    
    # ========== 其他参数 ==========
    pf_err: 0.05                     # 粒子滤波误差
    pf_z: 0.99                       # 粒子滤波置信度
    save_pose_rate: 0.5              # 位姿保存频率
    
    # 激光模型权重
    z_hit: 0.5                       # 命中权重
    z_rand: 0.5                      # 随机权重
    z_max: 0.05                      # 最大距离权重
    z_short: 0.05                    # 短距离权重
    
    # 话题配置
    scan_topic: scan                 # 激光扫描话题
    map_topic: map                   # 地图话题
```

### 不同场景的参数调整

**场景1：开阔空间（特征稀疏）**
```yaml
# 增加粒子数和激光范围
max_particles: 5000
min_particles: 1000
laser_max_range: 15.0
max_beams: 120
```

**场景2：狭窄走廊（特征密集）**
```yaml
# 减少粒子数，增加更新频率
max_particles: 1000
min_particles: 300
update_min_d: 0.1
update_min_a: 0.1
```

**场景3：快速运动**
```yaml
# 增加容差，减少更新频率
transform_tolerance: 2.0
update_min_d: 0.25
update_min_a: 0.25
alpha1: 0.4
alpha2: 0.4
alpha3: 0.4
alpha4: 0.4
```

**场景4：高精度要求**
```yaml
# 增加粒子数，减小更新阈值
max_particles: 8000
min_particles: 2000
update_min_d: 0.05
update_min_a: 0.05
pf_err: 0.01
```

## 参考资料

- [Nav2 Documentation](https://navigation.ros.org/)
- [AMCL Documentation](https://navigation.ros.org/configuration/packages/configuring-amcl.html)
- [Map Server Documentation](https://navigation.ros.org/configuration/packages/configuring-map-server.html)
- [FAST-LIVO2 GitHub](https://github.com/hku-mars/FAST-LIVO2)
- [FAST-LIVO2 Paper](https://arxiv.org/abs/2308.14346)

## 总结

使用FAST-LIVO2建图 + AMCL定位的方案结合了两者的优势：

✅ **建图阶段**：FAST-LIVO2提供高质量、全局一致的地图
✅ **导航阶段**：AMCL提供轻量、稳定的定位
✅ **资源消耗**：导航时CPU和内存占用低
✅ **灵活性**：支持多地图切换，地图可编辑
✅ **稳定性**：成熟的Nav2导航栈

这是当前机器人导航的最佳实践方案之一！
