clc
clear
close all

addpath('lib')

% Select state space  
ss = stateSpaceSE2;

% Build state validator
sv = validatorOccupancyMap(ss);

load workspace\mymap.mat

% Add map to state validator

sv.Map = map;

sv.ValidationDistance = 0.1;

ss.StateBounds = [sv.Map.XWorldLimits; sv.Map.YWorldLimits; [-pi pi]];

% Build planner using ss and sv
planner = planRRT(ss, sv);
planner.MaxConnectionDistance = 50;
planner.MaxIterations = 800;%限制最大迭代次数

% Continue planning after reaching goal 
planner.ContinueAfterGoalReached = true;

% Time limit for planning
 planner.MaxTime = 100;

% for mymap/mymapver2/pipemap/map2
  start = [200, 660, 0];
  goal = [600, 40, 0];

% for map2d 
%   start = [100, 500, 0];
%   goal = [850, 780, 0];

% for map 
%   start = [100, 550, 0];
%   goal = [450, 250, 0];

% for map2D_1
% start = [40, 40, 0];
% goal = [70, 40, 0];

% for map2D_2
% start = [30, 45, 0];
% goal = [80, 45, 0];

% for map2D_3
% start = [20, 10, 0];
% goal = [80, 90, 0];

% for map2D_4
% start = [20, 90, 0];
% goal = [80, 80, 0];

% for mapMaze_1
% start = [30, 70, 0];
% goal = [60, 30, 0];

% for mapMaze_2
% start = [10, 80, 0];
% goal = [90, 40, 0];


% for mapMaze_3
% start = [60, 60, 0];
% goal = [10, 70, 0];

% for mapMaze_4
% start = [10, 10, 0];
% goal = [10, 110, 0];

% Planning step
[pthObj, solnInfo] = plan(planner, start, goal);

% Display planning time and path length
disp(['Planning Time: ', num2str(solnInfo.ElapsedTime), ' seconds']);
disp(['Path Length: ', num2str(solnInfo.PathDistance)]);


% 2D Plot of generated nodes and final path
f1 = figure;
f1.Position = [400 200 600 500];
hold on
show(sv.Map)
plot(solnInfo.TreeData(:, 1), solnInfo.TreeData(:, 2), '.-')
plot(pthObj.States(:, 1), pthObj.States(:, 2), 'r-', 'LineWidth', 2)
plot(start(1), start(2), 'ko')
plot(goal(1), goal(2), 'ko')
title(" ")
hold off

% bezierOutput = bezierCurveSmoothing(pthObj);
% 
% f2 = figure;
% f1.Position = [400 200 600 500];
% hold on
% show(sv.Map)
% plot(bezierOutput.bx, bezierOutput.by, 'r-', 'LineWidth', 2)
% title("Path After Smoothing")
% hold off
