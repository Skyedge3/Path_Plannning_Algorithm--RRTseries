% filepath: /f:/机械臂/小论文/小论文代码/RRTseries_dev/main3d.m
clc; clear; close all;
addpath('lib')

%% 1. 定义工作空间范围和分辨率
% 注意：根据要求设置工作空间范围和分辨率
xLimits = [0, 60];  
yLimits = [0, 90];  
zLimits = [0, 120];  % 墙的高度为120，圆柱高度沿z轴
resolution = 1;      % 单位分辨率

% 生成网格
[xGrid, yGrid, zGrid] = ndgrid(xLimits(1):resolution:xLimits(2), ...
                               yLimits(1):resolution:yLimits(2), ...
                               zLimits(1):resolution:zLimits(2));

%% 2. 构建障碍物
% 初始化占用矩阵
occData = zeros(size(xGrid));

% --- 墙A ---
% 定义：墙A平行于 xoz 平面，位于 y=10，
% 宽度=60（以 x=30 为中心 ⇒ x∈[0,60]），高度=120（以 z=60 为中心 ⇒ z∈[0,120]）
% 墙A中心（30,10,60）设有一个半径为8的洞（在 x–z 平面上），该区域不占用
maskWallA = (yGrid == 10) & (xGrid >= 0 & xGrid <= 60) & (zGrid >= 0 & zGrid <= 120);
maskHoleA = (yGrid == 10) & (sqrt((xGrid-30).^2 + (zGrid-60).^2) <= 8);
wallA = maskWallA & ~maskHoleA;

% --- 墙B ---
% 定义：墙B与墙A尺寸一致，但位于 y=90，无洞
maskWallB = (yGrid == 90) & (xGrid >= 0 & xGrid <= 60) & (zGrid >= 0 & zGrid <= 120);
wallB = maskWallB;

% --- 中间圆柱 ---
% 定义：圆柱体放置在墙A与墙B之间，即 y=50，且圆柱的轴沿 z 方向，
% 在 x–y 平面，圆柱的横截面以 (30,50) 为中心，半径为7；
% 高度覆盖整个 z 方向范围 [0,120]
maskCylinder = (sqrt((xGrid-30).^2 + (yGrid-50).^2) <= 7) & (zGrid >= 0 & zGrid <= 120);

% 将三部分障碍物合并（占用标记为1）
occData( wallA | wallB | maskCylinder ) = 1;

%% 3. 构建 occupancyMap3D 并设置占用数据
map3D = occupancyMap3D(resolution);
allPoints = [xGrid(:), yGrid(:), zGrid(:)];
setOccupancy(map3D, allPoints, occData(:));

%% 4. 构建3D状态空间和验证器
% 使用 SE3 状态空间：状态格式 [x y z qw qx qy qz]
ss = stateSpaceSE3;
ss.StateBounds = [xLimits; yLimits; zLimits; [-1 1]; [-1 1]; [-1 1]; [-1 1]];

% 使用自定义的或内置的3D状态验证器
sv = validatorOccupancyMap3D(ss);
sv.Map = map3D;
sv.ValidationDistance = 1;

%% 5. 选择规划器并设置参数（假设 planRRT 已支持 SE3 规划）
planner = optPSOQRRTStar(ss, sv);
planner.MaxConnectionDistance = 50;
planner.MaxIterations = 450;
planner.ContinueAfterGoalReached = true;

%% 6. 设置起点和终点（7维状态：[x y z qw qx qy qz]）
% 起点：(30,0,60)；终点：(30,90,90)；四元数采用单位四元数 [1,0,0,0]
start = [30, 0, 60, 1, 0, 0, 0];
goal  = [30, 90, 90, 1, 0, 0, 0];

%% 7. 调用规划器进行路径规划
[pathObj, solnInfo] = planner.plan(start, goal);

% 绘制障碍物
hMap = show(map3D);
alpha(hMap, 0.6);  % 降低障碍物透明度

hold on; grid on; view(3);

% 绘制随机树
if isfield(solnInfo, 'TreeData') && ~isempty(solnInfo.TreeData)
    treeData = solnInfo.TreeData;
    hTree = plot3(treeData(:,1), treeData(:,2), treeData(:,3), '.-', 'MarkerSize', 6);
end

% 绘制规划路径（用加粗的红色线显示）
if isprop(pathObj, 'States') && ~isempty(pathObj.States)
    pathStates = pathObj.States;
    hPath = plot3(pathStates(:,1), pathStates(:,2), pathStates(:,3), 'r-', 'LineWidth', 4);
    uistack(hPath, 'top');  % 将路径置于顶层
else
    disp('规划路径为空');
end

% 绘制起点和终点
plot3(start(1), start(2), start(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor','g');
text(start(1) + 0.1, start(2) + 0.1, start(3) - 8, 'start', 'FontSize', 15, 'Color', 'k');
plot3(goal(1), goal(2), goal(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor','b');
text(goal(1) + 0.1, goal(2) + 0.1, goal(3) + 10, 'goal', 'FontSize', 15, 'Color', 'k');
% plot3(start(1), start(2), start(3), 'ko', 'MarkerSize', 8, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'none');
% plot3(goal(1), goal(2), goal(3), 'ko', 'MarkerSize', 8, 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'none');
title('');
drawnow;
hold off;

xlabel('');
ylabel('');
zlabel('');

% 计算并输出规划总时间、路径长度以及最大偏转角
if solnInfo.IsPathFound
    % 规划时间
    planningTime = solnInfo.ElapsedTime;
    
    % 路径长度（如果 solnInfo 中已经计算好，也可以重新计算）
    pathLength = solnInfo.PathDistance;
    
    % 计算最大偏转角（取路径中连续3个点，计算转角，单位为度）
    pathStates = pathObj.States;   % size: [N x 7]，前三列为位置信息
    maxTurningAngle = 0;
    if size(pathStates,1) >= 3
        for i = 1:size(pathStates,1)-2
            A = pathStates(i,1:3);
            B = pathStates(i+1,1:3);
            C = pathStates(i+2,1:3);
            v1 = B - A;
            v2 = C - B;
            % 防止除以零的情况
            if norm(v1) > 0 && norm(v2) > 0
                angle = acosd( dot(v1, v2) / (norm(v1)*norm(v2)) );
                maxTurningAngle = max(maxTurningAngle, angle);
            end
        end
    end
    
    disp(['最终路径长度：', num2str(pathLength)]);
    disp(['规划总时间：', num2str(planningTime), ' 秒']);
    disp(['最大偏转角：', num2str(maxTurningAngle), ' 度']);
else
    disp('未找到有效路径，无法计算指标');
end