# 手眼标定方法与坐标约定

## 坐标系

左腕相机固定在左夹爪末端时，求解的外参是 `T_gripper_camera`（相机 → 夹爪），与[参考说明](https://wcnfjfwof9lo.feishu.cn/wiki/KxB4wzyN6iSQwRkTpEJcY8tXnpd)中的 `T_left_gripper_from_cam_left` 含义一致。右腕相机按相同方式独立求解。相机光学坐标系、机器人末端坐标系、基座坐标系必须在采样、求解和部署中保持一致。

对于相机坐标系中的齐次点 `p_camera`：

```text
p_base = T_base_gripper × T_gripper_camera × p_camera
```

## 采集与求解

`checkerboard_pose_node.py` 使用相机图像和内参检测 **8 × 11 内角点**的棋盘格，按 **0.02 m** 方格边长计算 `T_camera_board`。`checkerboard_handeye_calibration.py` 采集与之对应的 `T_base_gripper`，通过 OpenCV 手眼标定求解 `T_gripper_camera`。采样应覆盖多种末端位置与方向，并避免连续保存近乎相同的姿态。

## 验证

固定棋盘在基座坐标系中的位姿应与机械臂姿态无关。对第 `i` 组样本计算：

```text
T_base_board,i = T_base_gripper,i × T_gripper_camera × T_camera_board,i
```

`validate_handeye_result.py` 统计这些位姿相对均值的平移与旋转残差，`plot_handeye_validation.py` 将每组样本的残差绘成曲线。该指标检查采样内部一致性；部署前还需在实际工作空间检查目标点、夹爪动作和碰撞距离。

`recompute_handeye_result.py` 可排除指定样本索引后重新求解。`visualize_handeye_application.py` 可把变换应用到实时图像与深度数据，生成投影和基座坐标系视图。
