# AlienGo 下排 D435 安装外参查询

查询日期：2026-11-15。目标为原厂机载下排 D435，相对于本项目 `base`（trunk 原点）的安装位姿。

## 结论

已找到公开 AlienGo＋Z1 工程模型中 D435 的明确安装数值；未找到足以确认本台原厂下排 D435 出厂外参的官方公开数据。下面是**第三方模型名义值**，不是本台实测或宇树出厂标定。

## 可追溯的模型参考

来源：[danisotelo/qm_door 的 robot.xacro](https://github.com/danisotelo/qm_door/blob/009a37d2c705c17097e8e22db79bfe112ffe24e0/qm_description/urdf/quadruped_manipulator/robot.xacro)。该模型以 AlienGo trunk 网格作为 `base`，D435 名称为 `camera2`，安装平移 `[0.32, 0.01, -0.035]` m、RPY `[0,0,0]` rad。

需要区分原点：[camera2.xacro](https://github.com/danisotelo/qm_door/blob/009a37d2c705c17097e8e22db79bfe112ffe24e0/qm_description/urdf/quadruped_manipulator/camera2.xacro) 将上述安装变换应用到 `camera2_bottom_screw_frame`，而非 `camera2_link`。该宏还给出螺孔到 link 的平移：

- X：`0.0149 − 0.0001 − 0.0042 = 0.0106` m。
- Y：`0.0175` m。
- Z：`0.025 / 2 = 0.0125` m。

各级安装旋转均为零，因此该模型内的合成结果为：

| 变换 | xyz（m） | rpy（rad） |
|---|---|---|
| base → camera2_bottom_screw_frame | `[0.3200, 0.0100, -0.0350]` | `[0,0,0]` |
| base → camera2_link | **`[0.3306, 0.0275, -0.0225]`** | **`[0,0,0]`** |

第二行由模型的两个固定变换合成得到。作者未将其声明为宇树原厂下排相机的出厂标定，也未确认与你的机器属于同一头部硬件配置；`camera2` 是该工程的命名，不能据此判定上下排。零RPY同样只是该模型设定。实机配置应对应实际驱动 `d435_link` 的原点，并核对坐标定义与相机序列号。

## 官方资料与排除项

- [宇树 AlienGo 开发说明](https://github.com/UnitreeSupport/Unitree_Docs/blob/master/docs/Aliengo/AlienGo.md) 给出 D435 型号、机载访问及使用方法，未列出机身到下排相机的六自由度安装外参。
- [官方 unitree_ros 的 AlienGo robot.xacro](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/aliengo_description/xacro/robot.xacro) 没有独立的 D435 安装 joint/TF；本地固定版本也未提供该安装链。
- 搜索到的 [wolf-setup issue #8](https://github.com/graiola/wolf-setup/issues/8) 有 `[0.3235,0,-0.066]` 的相机位置，但使用 ASUS 通用深度相机模型，不作为原厂 D435 外参依据。
- 其他实验室的外置/仿真 D435 配置存在不同位置和朝向，未将其混用为原厂下排参数。

下一步优先在接机后查找机载原厂配置或既有 TF 中的机身—相机安装关系，并对应到选中的下排相机。若能取得匹配的厂方数据，可直接用于核对；只有缺少可信安装关系时才需要重新测量或标定。本次查询未更改实机外参配置或运行代码。
