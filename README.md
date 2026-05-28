# g1_3d_nav_ros2

> Status: 2026-05-28 — verified end-to-end on G1 with cross-host RViz2 from a
> workstation. Image: `docker.io/zwterzt/g1_3d_nav_ros2:latest` SHA
> `137a5b46be62...`.

ROS 2 Humble native 3D localization runtime for the Unitree G1 Edu humanoid.
Single RMW (`rmw_zenoh_cpp`), no DDS bridge, no ros1_bridge.

## What this repo holds

- `3d_nav_g1/g1_ws/src/` — full deepglint source tree (FAST_LIO, open3d_loc)
  with all 2026-05-25 patches applied
- `3d_nav_g1/livox_ws/src/` — Livox MID360 ROS 2 driver source
- `3d_nav_g1/deps/open3d141/` — Open3D 0.14.1 headers + CMake config (binary
  libs separate, see `3d_nav_g1/deps/open3d141/README.md`)
- `tools/nav/` — G1-container-side launch + brake scripts
- `tools/mapping/` — G1-container-side mapping pipeline
- `tools/host_side/` — workstation-side scripts (RViz2 launchers, map editor)
- `tools/gotop/` — waypoint capture + nav + batch testing
- `maps/` — 2D occupancy grid (3D PCD is build product, not in git — see
  `maps/README.md`)
- `configs/` — RViz2 configs + `g1_host.txt` source-of-truth IP

The full set of patches is what makes the stack work — see `docs/DECISIONS.md`
for the architecturally significant decisions.

## Runtime topology

```
G1 (<G1 ip>)                                  workstation
┌─ 3d_nav_ros2 container ──────────────┐    ┌─ host install ─┐
│ image: g1_nav_final:latest           │    │ rmw_zenoh_cpp  │
│ net=host, ipc=host                   │    │ rviz2          │
│                                      │    └────────────────┘
│ [1/6] rmw_zenohd          :7448 ◄────┼──tcp/7448──┐
│ [2/6] livox_ros_driver2              │            │ (RMW=rmw_zenoh_cpp client
│ [3/6] fast_lio (FAST-LIO odometry)   │            │  + ZENOH_CONFIG_OVERRIDE)
│ [4/6] open3d_loc (ICP global loc)    │            │
│ [5/6] map_server (/map_2d)           │            │
│ [6/6] pointcloud_to_laserscan (/scan)│            │
└──────────────────────────────────────┘
```

In-container nodes share `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and connect to the
in-container Zenoh router on `tcp/127.0.0.1:7448`. Workstation RViz2 connects
to the same router across the network.

## Zenoh workflow

跨机器 ROS 2 通信全程走 Zenoh，不用 DDS。三件事记牢：

1. **G1 容器内**：`rmw_zenohd` 监听 `0.0.0.0:7448`；所有节点（fast_lio /
   open3d_loc / map_server / nav2 / etc.）用 `RMW_IMPLEMENTATION=rmw_zenoh_cpp`，
   作为 client 连本地 router `tcp/127.0.0.1:7448`。
2. **workstation 端**：`rviz2` 启动前 export 三件套——
   ```bash
   export RMW_IMPLEMENTATION=rmw_zenoh_cpp
   export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/<G1 ip>:7448"]'
   export ZENOH_ROUTER_CHECK_ATTEMPTS=10
   ```
   `7448` 是 G1 容器 zenohd 监听的固定端口（除非你在 G1 上跑了别的 zenohd 改了
   端口才需要改）。`<G1 ip>` 默认 `192.168.100.30`（见 `configs/g1_host.txt`）。
3. **环境刷新**：`ros2 daemon stop && ros2 daemon start && sleep 4` ——
   没这步第二次启动 ros2 cli 时 discovery 偶发飘忽。

`tools/host_side/g1_nav_loc_rviz2.sh` 和 `tools/host_side/mapping_rviz2.sh`
把上面三件套全做了；操作员只要 `bash` 一下脚本。

## Quickstart — one-off setup

### 1. Clone 仓库（workstation 端 + G1 端各一次）

```bash
[host] git clone https://github.com/leokkzzhzzz/g1_3d_nav_ros2.git
[host] cd g1_3d_nav_ros2

[G1]   ssh unitree@<G1 ip>
[G1]   git clone https://github.com/leokkzzhzzz/g1_3d_nav_ros2.git \
            /home/unitree/g1_3d_nav_ros2_repo
```

> G1 端 clone 路径**必须是** `/home/unitree/g1_3d_nav_ros2_repo`——这个路径
> 是容器 mount 的源（步骤 4），所有 `tools/` 通过这个 mount 进容器。

### 2. workstation 装 zenoh_cpp + rviz2

```bash
[host] sudo apt install ros-humble-rmw-zenoh-cpp ros-humble-rviz2
```

### 3. G1 端拉镜像

```bash
[G1] docker pull docker.io/zwterzt/g1_3d_nav_ros2:latest
[G1] docker tag  docker.io/zwterzt/g1_3d_nav_ros2:latest g1_nav_final:latest
```

### 4. G1 端创建 3d_nav_ros2 容器

```bash
[G1] docker run -d --name 3d_nav_ros2 \
        --network host --ipc host \
        -v /home/unitree/g1_3d_nav_ros2_repo:/g1_3d_nav_ros2 \
        -v /home/unitree/g1_3d_nav_ros2_repo/maps:/g1_3d_nav_ros2/maps \
        g1_nav_final:latest sleep infinity
```

容器启动后是个空 daemon，所有 `tools/` 通过 mount 立即可用，`git pull` 在
host 上做 → 容器内立即生效，无需 `docker cp`。

### 5. 准备 scans.pcd

新场地 → 跳到 [Mapping](#mapping) 自己建。
有现成 PCD（≥ 1 MB） → 放到 `/home/unitree/g1_3d_nav_ros2_repo/maps/scans.pcd`。

---

## Mapping

什么时候用：(a) 第一次给新场地建图、(b) 现场地图对不上现实需要重建。

**输出**（写到 host `/home/unitree/g1_3d_nav_ros2_repo/maps/`）：

| 文件 | 用途 |
|---|---|
| `scans.pcd` | 3D 全局点云，open3d_loc 加载用作 ICP 模板 |
| `accumulated_grid.pgm` + `.yaml` | 2D 占用栅格，map_server 加载发布到 `/map_2d`，nav2 静态层用 |

### 启动建图栈（4 节点）

```bash
# Window A — G1：建图栈
[G1] ssh unitree@<G1 ip>
[G1] docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/mapping/mapping_launch.sh
# 等 "=== MAPPING STACK READY (4 nodes, mapping mode) ==="
# 再等 ~10 秒 /tmp/fastlio.log 出现 "IMU Initial Done"
# 这期间 G1 必须站着别动

# Window B — workstation：可视化
[host] cd <repo>
[host] bash tools/host_side/mapping_rviz2.sh
```

实时看建图进度：

```bash
[G1] docker exec 3d_nav_ros2 tail -f /tmp/grid.log
# 每 5s 打印 frames=N ground=N obs=N grid=WxH
```

### 驱动 G1 走一圈

RC 手柄 + sport mode + 慢走。要点：

- 走遍所有未来要标 waypoint 的位置
- 每个门 / 走廊从两个方向都过一遍
- 在每个 waypoint 候选位置**停 5 秒**，让 fast_lio 累积稠密局部点云
- 原地慢转 360°，让 LiDAR 扫到周围
- 总共 5–15 分钟典型；短了有盲区

走完**回到起点**（floor tape 标记的 G1 起始位置）—— 这让后面 ICP 容易对齐。

### 保存：在 Window A 终端按 Ctrl+C

`mapping_launch.sh` 装了 SIGINT trap：你按 Ctrl+C 它**自动调** `mapping_save.sh`
保存，不用单独再跑命令（ADR-007）。

预期输出：

```
=== Ctrl+C received — running mapping_save.sh ===
[1/3] dumping 3D PCD via fast_lio /map_save ...
  ok: /g1_3d_nav_ros2/maps/scans.pcd (2382777 bytes)
[2/3] dumping 2D PGM via map_saver_cli ...
  ok: /g1_3d_nav_ros2/maps/accumulated_grid.pgm + .yaml
[3/3] fixing yaml image path ...

DONE. Files in /g1_3d_nav_ros2/maps/ :
 -rw-rw-r-- 1 1000 1000 2382777 ... scans.pcd
 -rw-rw-r-- 1 1000 1000 1175546 ... accumulated_grid.pgm
 -rw-rw-r-- 1 1000 1000     124 ... accumulated_grid.yaml
```

落盘位置（容器内 ↔ host 一一对应，同一份文件）：

| 容器内 | host 上 |
|---|---|
| `/g1_3d_nav_ros2/maps/scans.pcd` | `/home/unitree/g1_3d_nav_ros2_repo/maps/scans.pcd` |
| `/g1_3d_nav_ros2/maps/accumulated_grid.pgm` | `…/maps/accumulated_grid.pgm` |
| `/g1_3d_nav_ros2/maps/accumulated_grid.yaml` | `…/maps/accumulated_grid.yaml` |

PGM/yaml 可以 commit 进仓库当 canon。`scans.pcd` 不进 git（每场地独立、太大）。

### 验证 ICP fitness ≥ 0.7

切到导航栈（下节）后看：

```bash
[G1] docker exec 3d_nav_ros2 tail -50 /tmp/loc.log | grep fitness | tail -10
```

`reg_result.fitness` 持续 ≥ 0.7 才算建图成功。低于 0.7 → workspace 扫得不够、
重建。

---

## Map editing — `ros_map_edit`

mapping_save.sh 出来的 raw PGM 经常需要手工修：墙壁中间的雷达噪点要擦、画
虚拟墙限制 nav2、标记 region 等。`tools/host_side/map_edit/` 是一个 self-
contained 的 ROS 1 noetic Docker 镜像 + RViz panel，跑在 **workstation 端**。
完整文档见 [`tools/host_side/map_edit/README.md`](tools/host_side/map_edit/README.md)。

### 一次性准备（workstation 端）

```bash
[host] cd tools/host_side/map_edit
[host] docker build -t map_edit_rviz:latest .

[host] mkdir -p "$HOME/g1_maps"
[host] docker run -d --name map_edit_rviz \
        -e DISPLAY="$DISPLAY" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v "$HOME/g1_maps:/root/maps" \
        map_edit_rviz:latest
```

### 编辑 + 回传流程

```bash
# 1. 从 G1 拉地图到 workstation
[host] scp unitree@<G1 ip>:/home/unitree/g1_3d_nav_ros2_repo/maps/accumulated_grid.{pgm,yaml} \
        "$HOME/g1_maps/"
[host] sed -i 's|^image:.*|image: accumulated_grid.pgm|' "$HOME/g1_maps/accumulated_grid.yaml"

# 2. 启动编辑器
[host] bash tools/host_side/map_edit/start_map_edit.sh /root/maps/accumulated_grid.yaml

# 3. RViz2 里：
#    - MapEraser  → 黑/白擦除（改占用 / 自由 cell）
#    - VirtualWall → 画虚拟墙（两点画线）
#    - Region      → 画多边形区域
#    - 编辑完点绿色 "Save All Files" 按钮
#    保存到 $HOME/g1_maps/accumulated_grid.{pgm,yaml,json,_region.json}

# 4. 传回 G1（mode: trinary 是 ros_map_edit 漏写，要补；image: 改回容器内绝对路径）
[host] grep -q '^mode:' "$HOME/g1_maps/accumulated_grid.yaml" \
        || sed -i '2a mode: trinary' "$HOME/g1_maps/accumulated_grid.yaml"
[host] sed -i 's|^image:.*|image: /g1_3d_nav_ros2/maps/accumulated_grid.pgm|' \
        "$HOME/g1_maps/accumulated_grid.yaml"
[host] scp "$HOME/g1_maps"/accumulated_grid.{pgm,yaml,json} \
        unitree@<G1 ip>:/home/unitree/g1_3d_nav_ros2_repo/maps/

# 5. G1 端 hot-reload map_server（不用重启 nav 栈）
[G1] docker exec 3d_nav_ros2 bash -c '
    source /opt/ros/humble/setup.bash; source /botbrain_ws/install/setup.bash
    export RMW_IMPLEMENTATION=rmw_zenoh_cpp
    export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/127.0.0.1:7448\"]"
    ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap \
        "{map_url: /g1_3d_nav_ros2/maps/accumulated_grid.yaml}"'
```

---

## Localization + Navigation

End-to-end goal-driven motion verified 2026-05-26: RViz2 `2D Goal Pose` →
planner → controller → twist_mux → `g1_write_node` → SDK `LocoClient::Move()`
→ G1 walks.

### 启动顺序（3 个终端）

```bash
# Window A — G1：定位栈（6 节点）
[G1] ssh unitree@<G1 ip>
[G1] docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/nav/launch.sh
# 等 "=== ALL 6 NODES RUNNING ==="

# Window B — G1：Nav2 + twist_mux + g1_write_node（运动闭环）
[G1] ssh unitree@<G1 ip>
[G1] docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/nav/nav2_launch.sh
# 等 "=== STACK READY: G1 motion ENABLED ==="

# Window C — workstation：RViz2
[host] cd <repo>
[host] bash tools/host_side/g1_nav_loc_rviz2.sh
```

### 第一次必做：在 RViz2 设初始 pose

`open3d_loc` ICP 需要 G1 启动时位姿在 PCD 地图的 ~1 m 范围内。否则
`/localization_3d_confidence` 一直是 0、`map → odom` 不修正、fast_lio 长跑
会 drift 到非物理坐标。

启 nav 栈后在 RViz2 用 **2D Pose Estimate**（绿色箭头）→ 点击 G1 实际位置
+ 拖箭头对齐前进方向。`/localization_3d_confidence > 0.7` 后链路稳定，30
分钟静态运行不漂。

完成后 RViz2 用 **2D Goal Pose** 发目标，G1 走过去。

### Safety preconditions（操作员责任）

`nav2_launch.sh` 默认开启 motion。发 goal 前必须满足 4 条：

1. 操作员在场，能直接看到 G1
2. G1 周围 ≥ 1 m 净空，不在台阶边
3. RC 在手 — **L2 + B 是硬件刹车**（独立于 ROS 栈）
4. 至少一种软件刹车（下面的 soft_stop 或 estop）手边能调

### Brakes（软件刹车）

| 工具 | 行为 | 何时用 |
|---|---|---|
| `tools/nav/soft_stop.sh` | 取消所有 `/navigate_to_pose` goal → twist_mux fallback `cmd_vel_zero` → G1 **原地保持站立** | 常规"停一下"。立即可发新 goal |
| `tools/estop.sh` | 调 `/emergency_stop` 服务 → SDK `stop_move()` + `BALANCE_SQUAT_SQUAT_STAND` → G1 **停 + 蹲下**。再调一次复位站起 | 真紧急。Fail-passive：即使 balance 失效也以低姿态着地 |

```bash
[G1] docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/nav/soft_stop.sh   # 常规
[G1] docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/estop.sh           # 紧急
```

### Troubleshooting — `base_footprint frame does not exist`

`/tmp/nav2.log` 大量 `Invalid frame ID "base_footprint"`，RViz2 看到 goal
acks 但没 plan polyline、G1 不动。修法：

```bash
[G1] docker stop 3d_nav_ros2 && docker start 3d_nav_ros2
# 然后重 launch.sh + nav2_launch.sh
```

详见 Roadmap R-009（D-009 fork mount overlay 行为问题）。

---

## Waypoint testing — Gotop

跑 nav 后做"开 G1 到指定位置 → 看 nav2 真把它送到没"。三个工具，全在
`tools/gotop/`：

| 工具 | 用途 |
|---|---|
| `capture.sh` | 标记当前 G1 位姿到 yaml |
| `goto.sh`    | 单点导航到指定 label，输出误差 |
| `batch.sh`   | 批量跑多 label × 多轮，输出 markdown 报告 |

label 格式 `[A-Za-z][A-Za-z0-9_-]*`：`kitchen` / `door1` / `lab_corner_3`。
yaml 持久化在 `/g1_3d_nav_ros2/data/waypoints.yaml`（host 同步可见）。

### 1. 点位获取（capture）

需要 `launch.sh` 跑着（不需要 nav2_launch.sh）。

```bash
[G1] docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/capture.sh
```

REPL 命令：

```
> kitchen              ← RC 走 G1 到目标位置，等站稳，输入 label
> door1                ← 走到下一个，再标
> list                 ← 看已存的所有点
> del kitchen          ← 删一个
> rename kitchen cafe  ← 改名
> q                    ← 保存退出（Ctrl-D 也行）
```

每输入 label 立即写盘，Ctrl+C 不丢数据。同名 label 重复输入：< 30 cm 静默
更新；> 30 cm 弹确认（防误输）。

### 2. 点位导航（goto）

需要 `launch.sh` + `nav2_launch.sh` 都在跑。**操作员在场、RC 在手**。

```bash
[G1] docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/goto.sh
```

REPL：

```
goto> kit<TAB>     ← Tab 自动补全 label
goto> kitchen      ← G1 walk；结束输出 nav2 status + xy_err / yaw_err
goto> list         ← 列所有 label
goto> q            ← 干净退出
goto> q!           ← 立即软停 + 退出
^C                  ← 走的时候按 = 同 q!
```

每段（成功 / 失败）追加一行到 `/g1_3d_nav_ros2/data/goto_history.csv`：

```csv
timestamp,label,goal_x,goal_y,goal_yaw_deg,nav2_status,duration_s,reached_x,reached_y,reached_yaw_deg,xy_err_m,yaw_err_deg
2026-05-26T10:30:15,kitchen,1.2300,4.5600,90.00,SUCCEEDED,12.30,1.2400,4.5500,89.50,0.0141,-0.50
```

### 3. 点位精度（batch）

批量 + markdown 报告。每段问 `physical_sanity (y/n/skip)`（防 nav2 报
SUCCEEDED 但定位漂的 case）。

```bash
# yaml 里所有 × 3 轮，每轮顺序随机
[G1] docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/batch.sh \
        --all --rounds 3 --shuffle

# 指定 label
[G1] docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/batch.sh \
        --labels kitchen,door1,lab_corner --rounds 3

# 按 group（yaml 里手编 groups: 段）
[G1] docker exec -it 3d_nav_ros2 /g1_3d_nav_ros2/tools/gotop/batch.sh \
        --labels @kitchen_zone --rounds 3
```

输出 markdown 报告 `/g1_3d_nav_ros2/data/batch_report.md`：

- 每段一行：round / label / goal pose / reached pose / xy_err / yaw_err /
  duration / nav2 status / sanity
- 每个 label 汇总：成功率、xy_err 平均±std、yaw_err 平均±std

每次 batch **覆盖**这个文件；要保留先改名 `cp batch_report.md
batch_$(date +%F).md`。

### Waypoint 测试时的刹车

| 操作 | 效果 |
|---|---|
| Ctrl+C 在 goto/batch 终端 | 软停（取消 goal、零速、G1 站立、脚本退出） |
| 另一终端跑 `tools/nav/soft_stop.sh` | 同软停，但不退出 goto/batch 脚本 |
| `tools/estop.sh` | 紧急停 + 蹲下 |
| RC L2+B | 硬件刹车，独立于 ROS |

---

## Appendix

### Why mapping_launch.sh is separate

`tools/nav/launch.sh` 跑 6 节点（fast_lio + open3d_loc + map_server +
pcl2laserscan + zenoh + lidar），后三个加载 *现有* `scans.pcd` /
`accumulated_grid.pgm`。建图时这反而有害：

- `open3d_loc` ICP 配准旧 scans.pcd → G1 走出旧地图覆盖时 fitness=0、漂移
  污染 fast_lio odom
- `map_server` 发旧 `/map_2d` → 订阅方看错环境
- `pointcloud_to_laserscan` 为 nav2 local costmap，跟 mapping 无关

`tools/mapping/mapping_launch.sh` 只跑 4 节点：rmw_zenohd + LiDAR + fast_lio
(mapping mode) + grid_accumulator。是 ROS 1 三终端建图（livox + fast_lio +
ground_cloud_accumulator）的 ROS 2 一键等价。

### Network host management

仓库里 G1 IP 字面值出现 ~22 处，刻意不用模板 var（操作员复制粘贴优先）。
换站点：

```bash
[host] bash tools/rename_host.sh <NEW_IP>          # 重写所有引用
[host] git diff                                    # review
[host] git commit -am "rename G1 host: <OLD> -> <NEW>"

# 预览不动文件
[host] bash tools/rename_host.sh --dry-run <NEW_IP>
```

`configs/g1_host.txt` 是单一 source of truth（无 runtime 进程读它，纯给
`rename_host.sh` 当参照）。脚本排除 `docs/TEST_REPORTS/**`、README 顶部
`> Status:` 行、`configs/g1_host.txt` 自身（最后一步重写）。

### Configuration reference

#### `pointcloud_to_laserscan` parameters

| param | value | rationale |
|---|---|---|
| `target_frame` | `body` | reproject to robot body for nav |
| `transform_tolerance` | `0.01` | match ROS 1 |
| `min_height` | `-1.35` | include floor-level obstacles (G1 standing height ~1.3 m off ground) |
| `max_height` | `0.15` | ignore overhead clutter |
| `angle_min` / `max` | `±π` | full 360° |
| `angle_increment` | `0.007` | ~0.4° resolution |
| `range_min` / `max` | `0.2` / `100` | full LiDAR range |
| `use_inf` | `true` | mark out-of-range as inf for Nav2 |
| `inf_epsilon` | `1.0` | inf substitution |

#### `fast_lio` `mid360.yaml`

- `common.lid_topic: "/livox/lidar"` — 必须匹配 livox_ros_driver2 实际
  topic 名。`/livox/custom_msg` 不工作（fast_lio 卡 "Node init finished"）
- `common.imu_topic: "/livox/imu"`
- `mapping.extrinsic_T: [-0.011, -0.02329, 0.04412]`
- `mapping.extrinsic_R: identity`
- `mapping.extrinsic_est_en: false` — D-002 follow-through，固定外参防 EKF 漂移

#### `open3d_loc` `loc_param_g1.yaml` Kalman keys

用 **slash-style** keys (`kf_baselink2map/x: [...]`)。嵌套 YAML
(`kf_baselink2map: { x: [...] }`) 会被 C++ 静默忽略，因为 `declare_parameter`
用字面 slash 形式。

#### `/map` publisher QoS

`global_localization_node` 发 `/map` 用 `KeepLast(1) + TRANSIENT_LOCAL +
RELIABLE`。让晚到的 RViz2 拿到 latched PCD。ROS 2 默认 publisher 是 VOLATILE，
跟 ROS 1 latched 语义不一样。详 `patches/README.md`。

#### `accumulated_grid.yaml` thresholds (load-bearing)

`free_thresh: 0.196` + `mode: trinary` —— grid_accumulator 把 UNKNOWN cell
编码为 PGM 像素值 205，map_server 转换概率 `(255-205)/255 = 0.196`。若用
nav2 默认 `free_thresh: 0.25`，UNKNOWN cell 全部被误判 FREE → planner 路过
墙壁。`mapping_save.sh` 已 pin `--free 0.196 --occ 0.65 --mode trinary`。

### Known issues

| issue | impact | workaround |
|---|---|---|
| `ros2 cli` 偶发 context error | `topic hz` 等偶尔报错 | `ros2 daemon stop && ros2 daemon start` |
| `<defunct>` 僵尸进程 | 仅观感 | `docker restart 3d_nav_ros2` reap |
| RViz2 `tf_static TypeHashNotSupported` | 日志噪音（rmw_zenoh_cpp 0.1.8） | 忽略，TF 实际工作 |

### Versioning

- **2026-05-28** — operator-workstation toolchain + ros_map_edit map editor
  + threshold / PGM-row-flip bug fixes. Image:
  `docker.io/zwterzt/g1_3d_nav_ros2:latest` SHA `137a5b46be62...`（从 GCR
  迁到 Docker Hub）。Includes: `tools/` split into `nav/` + `host_side/`;
  ros_map_edit 操作员端 Docker 镜像（i18n + `/rviz/map_file` rosparam
  fallback patches); `mapping_save.sh` pin `free_thresh=0.196 mode=trinary`
  (修 UNKNOWN 误判 FREE); `ros_map_edit::savePGM` Y-flip fix (PGM 行序
  vs OccupancyGrid `data[0]=Y_min` 不一致)。

- **2026-05-25** — initial public release. Image: `g1_nav_final:latest`
  SHA `183e0426c630...`. Includes: open3d_loc PCD-path fix + `/scan` remap
  + `/map` QoS = TRANSIENT_LOCAL; launch.sh P3 (RMW=zenoh) refactor;
  pcl2laserscan ROS 1 parity; mid360.yaml `lid_topic` correction.

### License

Configurations and scripts: same as upstream — Apache 2.0 / BSD per
individual file. This repo aggregates configurations and patches against
deepglint's open3d_loc and FAST_LIO; consult those projects for their
respective licenses.

