# 地图编辑器

G1 跑完建图会生成 `maps/accumulated_grid.{pgm,yaml}`，但里面常常有鬼影、缺墙、需要圈禁区。这些只能人工 GUI 操作，所以**在你自己的电脑上跑**，不在 G1 上。

整个流程就是：

```
G1 出图  ──►  工作站编辑  ──►  G1 热重载
              （仓库 maps/）
```

本目录三样东西：

- `Dockerfile` — 工作站用的镜像（noetic + RViz + 已打补丁的 `ros_map_edit`）
- `start_map_edit.sh` — 启动脚本，处理 X11 转发、插件路径、清掉外面继承来的 `ROS_MASTER_URI`
- `ros_map_edit/` — 改过的 RViz 插件源码（见文末"打过的补丁"）

**先进仓库**（下面所有命令都假设你已经 cd 进来了）：

```bash
cd ~/g1_3d_nav_ros2
```

---

## 一、第一次准备（只做一次）

构建镜像：

```bash
docker build -t map_edit_rviz:latest tools/host_side/map_edit
```

创建容器，把仓库里的 `maps/` 挂进去（这样改完地图 `git diff` 就能看到改了啥）：

```bash
docker run -d --name map_edit_rviz -e DISPLAY="$DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -v "$PWD/maps":/root/maps map_edit_rviz:latest
```

容器跑的是 `sleep infinity`，启动脚本用 `docker exec` 进去干活。

> 之前用过旧版本（挂 `$HOME/g1_maps` 那种）的话，先 `docker rm -f map_edit_rviz` 删掉重建。

---

## 二、从 G1 拉地图

ssh 上 G1，把 `maps/accumulated_grid.{pgm,yaml}` 复制到本机仓库的 `maps/` 下，怎么传你随意（scp / rsync / U 盘都行）。

文件到位之后，G1 yaml 里 `image:` 写的是 G1 容器内的绝对路径，本机加载不到，改成相对路径：

```bash
sed -i 's|^image:.*|image: accumulated_grid.pgm|' maps/accumulated_grid.yaml
```

---

## 三、启动编辑器

```bash
tools/host_side/map_edit/start_map_edit.sh /root/maps/accumulated_grid.yaml
```

RViz 会弹出来。左边是 **File Management** 面板（绿色 **Save All Files** 按钮 + **Open Map** 按钮），工具栏多了 4 个工具：`MapEdit`、`VirtualWall`、`Region`、`MapEraser`。

如果面板没出来，看文末"常见问题"。

---

## 四、编辑

| 工具 | 干什么 | 怎么操作 |
|---|---|---|
| **MapEraser** | 涂改栅格（占据/空闲/未知） | 左键画黑（占据）、右键画白（空闲）、按住拖动连续涂 |
| **VirtualWall** | 画两点虚拟墙，存到 `<图>.json` | 左键点两次定两端、右键取消当前墙 |
| **Region** | 圈一块命名区域，存到 `<图>_region.json` | 左键点多边形顶点、双击闭合 |
| **MapEdit** | 模式切换器，决定上面哪个工具激活 | — |

笔刷大小、墙的颜色宽度等，在右边 **Tool Properties** 面板调。

---

## 五、保存

点左边绿色的 **Save All Files**。会在 yaml 同目录写四个文件（同名覆盖）：

- `accumulated_grid.yaml` — 配置
- `accumulated_grid.pgm` — 图像（带你改过的内容）
- `accumulated_grid.json` — 虚拟墙（没画就是 `{"vws": []}`）
- `accumulated_grid_region.json` — 区域（没画就是 `{"regions": []}`）

弹个对话框告诉你存了啥。容器里写的是 `/root/maps/`，宿主机仓库的 `maps/` 立刻能看到，`git diff` 就是这次的改动。

---

## 六、推回 G1

把 yaml 改回 G1 能用的状态。`mode: trinary` 编辑器保存时会丢，必须加回来（默认值万一变了会出大事），`image:` 路径也要改回 G1 的绝对路径：

```bash
grep -q '^mode:' "$H/accumulated_grid.yaml" || sed -i '2a mode: trinary' "$H/accumulated_grid.yaml"
```

```bash
sed -i 's|^image:.*|image: /g1_3d_nav_ros2/maps/accumulated_grid.pgm|' "$H/accumulated_grid.yaml"
```

四个文件传回去（region.json 没画过就跳过）：

```bash
scp maps/accumulated_grid.{pgm,yaml,json} "unitree@<g1 ip>":/home/unitree/g1_3d_nav_ros2_repo/maps/
```

```bash
scp maps/accumulated_grid_region.json "unitree@<g1 ip>":/home/unitree/g1_3d_nav_ros2_repo/maps/ 2>/dev/null || true
```

---
