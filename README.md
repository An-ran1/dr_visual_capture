# 六轴机械臂 RGB-D 视觉抓取系统

基于 ROS Noetic、大然桌面六轴机械臂与 Intel RealSense D405 RGB-D 相机的视觉抓取项目。项目保留原有抓取闭环，并将感知和坐标转换链路升级为“手眼标定 + 实例分割 + 点云融合 + 障碍感知”的可验证方案。

## 原始项目资料

[机器人系统课程设计报告模板](https://github.com/user-attachments/files/26588663/default.docx)

<img width="191" height="162" alt="机械臂与相机安装" src="https://github.com/user-attachments/assets/636a9190-90e8-42f5-8da0-b24fb9f60df8" />

<img width="433" height="317" alt="原始 ROS 节点流程" src="https://github.com/user-attachments/assets/b348779f-9cfe-4ba3-b202-9d760c7a4e19" />

演示视频：

https://github.com/user-attachments/assets/958b77d0-c741-4365-88f4-a5dfdcdb564c

## 系统流程

原始闭环为：

```text
SSD-Mobilenet 检测框 -> 框中心单点深度 -> 固定相机偏移 -> 逆运动学 -> 抓取与复位
```

升级后的流程为：

```text
RGB-D 图像 + 相机内参
  -> YOLOv8-Seg 实例掩膜
  -> 掩膜/深度融合与目标点云
  -> 异常点过滤 + PCA 抓取主轴估计
  -> ChArUco 手眼标定得到 gripper_T_camera
  -> 障碍净空判断：直取 / 抬高预抓取 / 拒绝
  -> 既有逆解、关节限位、夹爪控制和复位
```

原有 `dr_robot_object_grasp.py` 继续负责机械臂控制。新增节点先发布标准 ROS 位姿与点云话题，在连接真机控制前可通过 RViz 和 rosbag 验证。

## 主要改进

| 模块 | 原始方案 | 升级方案 | 工程收益 |
| --- | --- | --- | --- |
| 二维感知 | SSD-Mobilenet 检测框 | YOLOv8-Seg 实例掩膜 | 区分相邻目标，减少检测框内背景像素干扰。 |
| 深度定位 | 取检测框中心单点深度 | 掩膜内有效深度点反投影 | 降低深度空洞、边缘点和中心点落到背景导致的误差。 |
| 目标姿态 | HSV、轮廓和霍夫直线规则 | 过滤后点云中心 + PCA 主轴 | 基于三维几何估计目标中心和夹爪平面抓取方向。 |
| 相机到机械臂转换 | 固定安装参数与坐标轴重排 | ChArUco 多位姿 `AX=XB` 手眼标定 | 外参可复现、可保存，并可用误差数据验证。 |
| 障碍应对 | 无显式障碍策略 | 净空圆柱策略：直取、抬高或拒绝 | 在高障碍接近目标时不下发危险抓取命令。 |

当前仓库不虚构精度、成功率或实时性结果。所有指标必须在相机、机械臂和目标物实测后填写。

## 核心算法

### 手眼标定

`node/handeye_calibrate.py` 使用 OpenCV Tsai 方法求解眼在手上的变换 `gripper_T_camera`。将 ChArUco 标定板固定在机械臂基座坐标系内；每次采集时记录机器人 `base_T_gripper`，由图像估计标定板相对于相机的位姿，通过相对运动关系求解：

```text
A X = X B
```

脚本仅保留至少识别到 6 个 ChArUco 角点的样本，并输出外参与跨位姿的标定板位置一致性误差。建议采集 12-20 组具有平移和旋转变化的位姿，另留至少 5 组未参与求解的位姿评估测试误差。

```bash
rosrun dr_robot_object_follower handeye_calibrate.py \
  --camera config/camera_intrinsics.yaml \
  --poses config/handeye_samples.yaml \
  --images data/handeye_captures \
  --output config/gripper_T_camera.yaml
```

输入格式示例见 `config/camera_intrinsics.example.yaml` 与 `config/handeye_samples.example.yaml`。完成验证后，控制节点应加载该 YAML 外参，并替换旧版硬编码的 `pl_camera` 偏移量。

### YOLOv8-Seg 与深度掩膜融合

`node/yolov8_seg_pointcloud.py` 使用 Ultralytics YOLOv8-Seg 获取实例掩膜，保留掩膜范围内的对齐深度，去除无效量程、偏离中位深度的点和距离离群点，再根据 `CameraInfo` 内参反投影为三维点：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```

随后对 XY 点云做 PCA/SVD，主成分方向作为平行夹爪的抓取朝向。节点发布：

- `~object_cloud`：过滤后的目标点云
- `~grasp_candidate`：目标中心和抓取候选姿态
- `~mask_overlay`：掩膜叠加图，用于检查分割质量

预训练 COCO 权重不能代表实验室零件上的效果。应采集真实目标的掩膜数据，对 `yolov8s-seg` 微调并记录 mask mAP。

### 基于障碍净空的抓取策略

`node/grasp_strategy.py` 检查目标周围可配置净空圆柱内的障碍点：无干扰时发布“预抓取 -> 抓取”两阶段位姿；低障碍时提高预抓取高度；障碍高度超过安全阈值时拒绝发布抓取计划。这是便于验证和解释的基线策略。

进一步工程化时，应将过滤后的场景点云转为 MoveIt 碰撞对象，进行整条机械臂轨迹的碰撞检测，而不只检查末端路径。

## 安装与运行

推荐环境：Ubuntu 20.04、ROS Noetic、Python 3、带 `aruco` 模块的 OpenCV、Intel RealSense SDK 和支持 CUDA 的主机或 Jetson。

```bash
pip3 install ultralytics pyyaml
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch dr_robot_object_follower yolov8_seg_grasp.launch model:=/path/to/best.pt
```

部署阶段可将模型导出为 ONNX/TensorRT。记录延迟时必须同时记录模型版本、输入分辨率、推理精度和硬件平台。

## 效果评估与对比方式

在相同的 20-30 组场景下对原始方案和升级方案录制 rosbag 并对比：单目标、相邻目标、部分遮挡、反光/深度空洞、低/中/高障碍。目标类别固定，物体姿态随机；统一以“物体被提升 50 mm 且保持 3 秒”为抓取成功标准。报告均值、标准差、试验次数和失败案例。

| 指标 | 原始方案记录 | 升级方案记录 | 用途 |
| --- | --- | --- | --- |
| 标定 | 留出测试点定位误差（mm） | 留出测试点定位误差（mm） | 验证相机-机械臂变换是否可信。 |
| 感知 | 三维中心误差、有效深度率 | 三维中心误差、有效深度率 | 衡量掩膜级深度融合的收益。 |
| 抓取 | 各类场景成功数/总次数 | 各类场景成功数/总次数 | 体现端到端抓取价值。 |
| 安全 | 碰撞次数、危险任务拒绝数 | 碰撞次数、危险任务拒绝数 | 评估障碍策略的有效性和保守性。 |
| 实时性 | 图像输入到候选位姿的延迟 | 图像输入到候选位姿的延迟 | 确认升级后仍可在线运行。 |

完成实测后可写出类似结论：

> 相比检测框中心深度，掩膜级融合将目标中心误差从 **[A] mm** 降至 **[B] mm**；在定义的测试集上，抓取成功率由 **[C]/[N]** 提升至 **[D]/[N]**，手眼标定留出测试误差为 **[E] mm**。

其中所有占位符必须替换为日志、rosbag 和标定 YAML 支持的实测数据。

## 简历表述

实机验证前可使用：

> 负责六轴机械臂 RGB-D 视觉抓取系统开发：基于 ROS Noetic 实现 ChArUco 手眼标定（AX=XB）、YOLOv8-Seg 与对齐深度图掩膜级点云融合，通过 PCA 估计物体三维中心与抓取朝向，并设计基于障碍净空的预抓取/拒绝策略；对接逆运动学、关节限位及夹爪控制接口，构建“感知-定位-规划-抓取”闭环。

完成评估后可补充：

> 开发六轴机械臂 RGB-D 视觉抓取系统，采用 ChArUco AX=XB 完成眼在手上外参标定（留出测试误差 **[E] mm**），融合 YOLOv8-Seg 实例掩膜和对齐深度估计三维中心及抓取轴；在遮挡和障碍场景测试中，抓取成功率由 **[C]/[N]** 提升至 **[D]/[N]**。

不要在没有标定文件、实验协议、rosbag 和实测结果的情况下，仅凭代码写“已完成高精度手眼标定”“动态避障抓取”或具体提升百分比。

## 后续工作

1. 将标定外参加入 TF：`base -> gripper -> camera`，经测试后去除旧版 `pl_camera` 常量。
2. 采集真实目标掩膜数据并微调 YOLOv8-Seg，报告 mask mAP。
3. 分割桌面和障碍物，将点云转为 MoveIt 碰撞对象，进行全臂碰撞检测。
4. 在 `~grasp_plan` 接入硬件控制前，增加状态机、运动完成确认与夹爪力反馈检查。
