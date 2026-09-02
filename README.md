# RGB-D Visual Grasping for a Six-Axis Robot

## Original project material

The system uses a DaRan desktop six-axis arm and an Intel RealSense D405 RGB-D
camera.  The original project report template, installation photograph and
demonstration video remain available below.

[Robot-system course project report template](https://github.com/user-attachments/files/26588663/default.docx)

<img width="191" height="162" alt="Installed camera and arm" src="https://github.com/user-attachments/assets/636a9190-90e8-42f5-8da0-b24fb9f60df8" />

<img width="433" height="317" alt="Original ROS node workflow" src="https://github.com/user-attachments/assets/b348779f-9cfe-4ba3-b202-9d760c7a4e19" />

https://github.com/user-attachments/assets/958b77d0-c741-4365-88f4-a5dfdcdb564c

This ROS Noetic project upgrades the original closed-loop pipeline from
`SSD-Mobilenet box -> box-centre depth -> fixed camera offset -> inverse
kinematics` to a calibrated, mask-level RGB-D grasping pipeline:

```text
RGB-D + camera intrinsics
  -> YOLOv8-Seg instance mask
  -> mask/depth fusion and object point cloud
  -> outlier filtering + PCA grasp axis
  -> calibrated gripper_T_camera transform
  -> obstacle-aware pre-grasp / grasp / reject decision
  -> existing IK, joint limits, gripper and return motion
```

The existing `dr_robot_object_grasp.py` keeps responsibility for arm control.
The new nodes publish standard ROS poses and clouds first, so they can be
validated before connecting them to the physical arm.

## What changed

| Capability | Original pipeline | Upgraded pipeline | Practical effect |
| --- | --- | --- | --- |
| 2D perception | SSD-Mobilenet bounding box | YOLOv8-Seg instance mask | Separates adjacent targets and avoids background pixels inside the box. |
| Depth estimate | One depth value at box centre | Hundreds of valid mask-depth points | Less sensitive to depth holes, edge pixels and a centre point falling on background. |
| Object pose | HSV contour/Hough-line heuristic | Filtered 3D centroid plus PCA principal axis | Class-independent object centre and in-plane grasp orientation. |
| Camera-arm transform | Fixed installation parameters and axis rearrangement | ChArUco multi-pose AX=XB hand-eye calibration with validation report | Transform is measurable, repeatable and saved as an extrinsic YAML. |
| Obstacle response | No explicit obstacle decision | Clearance-cylinder policy: direct, raised approach, or reject | Prevents unsafe commands near tall obstacles; publishes a two-stage approach. |

No performance figure is claimed until it is measured on the target camera,
robot and object set.  The code produces the needed calibration and runtime
outputs so the claims below can be filled with real data.

## Algorithms and engineering choices

### 1. Eye-in-hand calibration

`node/handeye_calibrate.py` estimates `gripper_T_camera` with OpenCV's Tsai
hand-eye solver.  A ChArUco board remains stationary in the robot base frame;
for each capture the robot records `base_T_gripper`, while the board pose is
estimated from the image.  The relative-motion relationship is the standard
hand-eye equation `A X = X B`.

The script retains only captures with at least six interpolated ChArUco corners
and writes the transform plus cross-pose board-position consistency in mm.  Use
12-20 poses spanning translations and rotations, then reserve at least five
unseen poses to report test-point error.  Do not copy the calibration error
into a resume until it has been measured on held-out poses.

```bash
rosrun dr_robot_object_follower handeye_calibrate.py \
  --camera config/camera_intrinsics.yaml \
  --poses config/handeye_samples.yaml \
  --images data/handeye_captures \
  --output config/gripper_T_camera.yaml
```

`config/*.example.yaml` documents both input schemas.  The control node should
load `gripper_T_camera.yaml` instead of the legacy hard-coded `pl_camera`
offset after the transform convention has been verified on the robot.

### 2. YOLOv8-Seg and depth-mask fusion

`node/yolov8_seg_pointcloud.py` uses the Ultralytics YOLOv8 segmentation API.
It selects an instance mask, intersects it with aligned RealSense depth, rejects
invalid range values, removes points far from median depth, suppresses the
outer 10% by distance, and back-projects remaining pixels using `CameraInfo`:

```text
X = (u - cx) * Z / fx,  Y = (v - cy) * Z / fy,  Z = depth(u, v)
```

PCA/SVD of the resulting XY cloud gives a stable primary direction for a
parallel gripper.  It publishes `~object_cloud`, `~grasp_candidate` and
`~mask_overlay`, enabling RViz inspection and bag-file evaluation.  Fine-tune
`yolov8s-seg` on the actual target classes; pretrained COCO weights alone are
not evidence of performance on lab-specific parts.

### 3. Obstacle-aware grasp policy

`node/grasp_strategy.py` checks obstacle points in a configurable horizontal
clearance cylinder around the candidate.  It publishes `[pre-grasp, grasp]` for
direct targets, raises the approach over low obstacles, and rejects candidates
when obstacle height exceeds the safety threshold.  This is an intentionally
auditable baseline.  For a production-grade planner, feed the filtered cloud as
collision objects into MoveIt and validate the full arm trajectory, not only
the tool-centre path.

## Installation and launch

Target platform: ROS Noetic, Python 3, OpenCV with `aruco`, a RealSense RGB-D
camera, and a CUDA-capable host or Jetson for real-time inference.

```bash
pip3 install ultralytics pyyaml
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch dr_robot_object_follower yolov8_seg_grasp.launch model:=/path/to/best.pt
```

The model can later be exported to ONNX/TensorRT for deployment.  Keep model
version, input resolution, precision and hardware in every latency report.

## Evaluation protocol and expected improvement

Record a rosbag for the original and upgraded pipeline under the same 20-30
scenes: isolated objects, adjacent objects, partial occlusion, reflective/depth
hole cases, and low/medium/high obstacles.  Use fixed target classes and random
object poses.  Report all results as `mean +/- std`, number of trials, and the
same success criterion: object lifted 50 mm and retained for 3 seconds.

| Metric | Legacy measurement | Upgraded measurement | Why it matters |
| --- | --- | --- | --- |
| Calibration | held-out target position error (mm) | held-out target position error (mm) | Demonstrates that the transform, rather than a fixed offset, is valid. |
| Perception | 3D centre error / valid depth rate | 3D centre error / valid depth rate | Measures benefit of mask-level depth fusion. |
| Grasp | successes / attempts by scene | successes / attempts by scene | Shows end-to-end value, not only detector accuracy. |
| Safety | collisions or rejected unsafe plans | collisions or rejected unsafe plans | Evaluates obstacle policy and conservatism. |
| Runtime | RGB input to candidate pose latency | RGB input to candidate pose latency | Ensures the upgrade remains usable online. |

Suggested honest conclusion after measurement: "Compared with box-centre depth,
mask-level fusion reduced target-centre error from **[A] mm** to **[B] mm** and
improved grasp success from **[C]/[N]** to **[D]/[N]** under the defined test
set; calibration held-out error was **[E] mm**."  Replace every placeholder
with a logged result and keep the bag/calibration YAML as evidence.

## Resume wording

Use only the first version before real-hardware validation:

> Built a ROS Noetic RGB-D visual grasping pipeline for a six-axis robot;
> implemented ChArUco hand-eye calibration (AX=XB), YOLOv8-Seg/depth-mask point
> cloud fusion, PCA grasp-axis estimation and an obstacle-clearance pre-grasp
> policy; integrated the candidate output with IK, joint-limit and gripper
> control interfaces.

After completing the evaluation protocol, replace it with:

> Developed a ROS Noetic RGB-D grasping system for a six-axis robot. Calibrated
> eye-in-hand extrinsics with ChArUco AX=XB (**[E] mm** held-out error), and
> fused YOLOv8-Seg masks with aligned depth to estimate 3D centroids and grasp
> axes. An obstacle-aware pre-grasp policy improved grasp success from
> **[C]/[N]** to **[D]/[N]** across occlusion and obstacle test scenes.

Avoid claiming generic "hand-eye calibration", "dynamic obstacle grasping", or
an improvement percentage without the calibration YAML, rosbag, test protocol
and measured result to support it.

## Next engineering steps

1. Add the calibrated transform to TF (`base -> gripper -> camera`) and remove
   legacy `pl_camera` constants only after held-out validation.
2. Fine-tune and validate YOLOv8-Seg on collected object masks; record mask
   mAP, not only box mAP.
3. Segment support plane and obstacles from the full scene cloud, then create
   MoveIt collision objects for whole-arm collision checking.
4. Add a state machine and motion completion/force checks before connecting
   `~grasp_plan` to the hardware controller.
