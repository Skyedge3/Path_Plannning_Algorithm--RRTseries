# RRT系列路径规划算法

## 项目简介

本仓库实现了一系列基于RRT（Rapidly-exploring Random Tree，快速探索随机树）的路径规划算法，用于机器人导航和运动规划。代码支持2D和3D环境下的路径规划，并集成了多种优化策略，如RRT、RRT*、QRRT*、PSORRT*等，同时提供了贝塞尔曲线平滑处理功能。

## 算法介绍

本项目实现的算法包括：

- **RRT**：随机采样的快速探索树算法
- **RRT***：带路径优化的RRT算法，能生成渐进最优路径
- **QRRT***：Quick-RRT*算法，提供祖先节点深度搜索功能
- **PSORRT***：结合粒子群优化(PSO)的RRT*算法，提高采样效率
- **C_QRRT***：带角度约束的QRRT*算法，限制路径转角
- **C_PQRRT***：带角度约束的PSOQRRT*算法，结合PSO优化随机采样过程

- **MORE**：更多RRT-based算法持续更新中...

## 依赖环境

- MATLAB (推荐版本 R2019b 或更高版本)
- MATLAB 机器人系统工具箱 (Robotics System Toolbox)
- MATLAB 导航工具箱 (Navigation Toolbox)

## 代码结构

```
.
├── lib/                       # 算法实现库
│   ├── bezierCurveSmoothing.m # 贝塞尔曲线平滑处理
│   ├── checkIfGoalIsReached.m # 目标到达检测
│   ├── planC_QRRTStar.m       # 带角度约束的QRRT*算法
│   ├── planCP_QRRTStar.m      # CP_QRRT*算法
│   ├── planPSOQRRTStar.m      # PSOQRRT*算法
│   ├── planPSORRTStar.m       # 结合PSO的RRT*算法
│   ├── planQRRTStar.m         # QRRT*算法实现
│   ├── planRRT.m              # RRT算法
│   ├── planRRTStar.m          # RRT*算法
│   └── sphere3D.m             # 3D距离计算函数
├── workspace/                 # 地图数据
│   ├── map.mat                # 多种测试地图数据
│   ├── obtainMAP.m            # 地图生成工具，可将png转换为.mat格式
│   └── *.png                  # 地图.png文件
├── main.m                     # 2D路径规划主程序
├── main3d.m                   # 3D路径规划主程序
```

## 使用方法

### 2D路径规划

1. 打开MATLAB并进入项目根目录
2. 运行`main.m`脚本进行2D环境下的路径规划：

```matlab
>> main
```

### 3D路径规划

1. 打开MATLAB并进入项目根目录
2. 运行`main3d.m`脚本进行3D环境下的路径规划：

```matlab
>> main3d
```

## 参数配置

主要配置参数位于`main.m`和`main3d.m`文件中，包括：

- `planner.MaxConnectionDistance`：节点连接的最大距离
- `planner.MaxIterations`：最大迭代次数
- `planner.ContinueAfterGoalReached`：到达目标后是否继续规划
- `planner.MaxTime`：规划的最大时间限制（秒）
- `planner.GoalBias`：向目标点偏置的概率

对于QRRT*等算法：
- `planner.ParentDepthSearch`：父节点深度搜索级别
- `planner.ParentDepthRewire`：重连父节点深度级别
- `planner.MaxDeflectionAngle`：最大允许偏转角（弧度）

对于PSO相关算法：
- `planner.InertiaWeight`：PSO惯性权重
- `planner.PopulationSize`：粒子群大小
- `planner.Iterations`：PSO迭代次数

## 地图配置

`workspace`目录包含多种测试地图。要切换地图，请修改`main.m`或`main3d.m`中的地图加载部分：

```matlab
% 在main.m中
load workspace\mymap.mat  % 可替换为其他地图

% 在main3d.m中修改障碍物定义部分
```

## 路径平滑

本项目提供贝塞尔曲线平滑处理功能，可在`main.m`中取消相关注释以启用：

```matlab
bezierOutput = bezierCurveSmoothing(pthObj);

f2 = figure;
f1.Position = [400 200 600 500];
hold on
show(sv.Map)
plot(bezierOutput.bx, bezierOutput.by, 'r-', 'LineWidth', 2)
title("路径平滑后")
hold off
```

## 功能展示

运行程序后，将显示：
1. 规划环境与障碍物
2. 探索树结构
3. 最终规划路径
4. 路径规划时间和路径长度
5. 路径最大偏转角