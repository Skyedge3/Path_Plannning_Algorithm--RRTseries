function isReached = checkIfGoalIsReached(planner, goalState, newState)
% 检查是否到达目标：只比较位置（前三维），因为状态为 [x y z roll pitch yaw]
isReached = false;
positionThreshold = 5;  % 允许的位姿误差（单位与地图一致）
delta = newState(1:3) - goalState(1:3);
if norm(delta) < positionThreshold
    isReached = true;
end
end

