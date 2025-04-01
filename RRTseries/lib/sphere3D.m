function r = sphere3D(x, y, z, goal)
    % 计算三维状态与目标状态之间的平方误差
    r = 0.8*(x - goal(1)).^2 + 0.8*(y - goal(2)).^2 + 0.8*(z - goal(3)).^2;
end