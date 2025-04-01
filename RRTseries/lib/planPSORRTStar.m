% filepath: /f:/机械臂/小论文/小论文代码/RRTseries_dev/lib/planPSORRTStar.m
classdef planPSORRTStar < planRRTStar

   properties
        InertiaWeight           % 粒子当前速度的惯性权重
        InertiaDamping          % 惯性权重的衰减系数
        LearningCoefficients    % 学习系数，用于局部最优（pBest）和全局最优（gBest）
        PopulationSize          % 粒子群规模
        Iterations              % 粒子群优化的迭代次数
        CostFunction            % 成本函数，用于评估粒子位置质量
   end
  
   methods
       function obj = planPSORRTStar(stateSpace, stateValidator)
           obj@planRRTStar(stateSpace, stateValidator)
           
           obj.InertiaWeight = 0.5;
           obj.InertiaDamping = 0.3;
           obj.LearningCoefficients = [0.1 0.1];
           obj.PopulationSize = 15;
           obj.Iterations = 4;
           % 修改成本函数为适用于三维规划的 sphere3D
           obj.CostFunction = @sphere3D;
       end
       
       function [pathObj, solnInfo] = plan(obj, start, goal)
           tic;
           obj.CurrentGoalState = goal;
           tentativeGoalIds = [];
           
           % create search tree
           treeInternal = nav.algs.internal.SearchTree(start, obj.MaxNumTreeNodes);
           treeInternal.setBallRadiusConstant(obj.BallRadiusConstant);
           treeInternal.setMaxConnectionDistance(obj.MaxConnectionDistance);
           
           if obj.GoalReachedFcn(obj, start, goal)
               pathObj = navPath(obj.StateSpace, [start; goal]);
               solnInfo = struct();
               solnInfo.IsPathFound = true;
               solnInfo.ExitFlag = obj.GoalReached;
               solnInfo.NumNodes = 1;
               solnInfo.NumIterations = 0;
               solnInfo.TreeData = [start; nan(1,obj.StateSpace.NumStateVariables); start; goal; nan(1,obj.StateSpace.NumStateVariables)];
               solnInfo.ElapsedTime = toc;
               solnInfo.PathDistance = obj.StateSpace.distance(start, goal);
               return;
           end
           
           pathFound = false;
           statusCode = obj.Unassigned;
           numIterations = 0;
           
           for k = 1:obj.MaxIterations
               samples = getRandomSample(obj);
               validIdx = obj.StateValidator.isStateValid(samples) == 1;
               validStates = samples(validIdx, :);
               if isempty(validStates)
                % 若无有效状态，直接返回 sampleList 中第一个样本
                randState = samples(1, :);
            else
                randState = validStates(randi(size(validStates, 1)), :);
            end
               
               if rand() < obj.GoalBias
                   randState = goal;
               end

               [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal);
                   
               if statusCode == obj.GoalReached
                   pathFound = true;
                   tentativeGoalIds = [tentativeGoalIds, newNodeId];
                   numIterations = k;
                   break;
               end
               
               if statusCode == obj.GoalReachedButContinue
                   pathFound = true;
                   tentativeGoalIds = [tentativeGoalIds, newNodeId];
               end
               
               if statusCode == obj.MaxNumTreeNodesReached
                   numIterations = k;
                   break;
               end
               
               elapsedTime = toc;
               if elapsedTime > obj.MaxTime
                   statusCode = obj.MaxTimeElapsed;
                   numIterations = k;
                   break;
               end              
           end 
           
           if numIterations == 0
               numIterations = obj.MaxIterations;
           end
           
           treeData = treeInternal.inspect();
           numNodes = treeInternal.getNumNodes();
           exitFlag = statusCode;
           if statusCode > obj.MotionInCollision && statusCode < obj.Unassigned
               exitFlag = obj.MaxIterationsReached;
           end
           
           if pathFound
               costBest = inf;
               idBest = -1;
               for j = 1:length(tentativeGoalIds)
                   nid = tentativeGoalIds(j);
                   c = treeInternal.getNodeCostFromRoot(nid);
                   if c < costBest
                       idBest = nid;
                       costBest = c;
                   end
               end
               pathStates = treeInternal.tracebackToRoot(idBest);
               pathObj = navPath(obj.StateSpace, flip(pathStates'));
               pathLength = treeInternal.getNodeCostFromRoot(idBest);
           else
               pathObj = navPath(obj.StateSpace);
               pathLength = nan;
           end
           
           solnInfo = struct;
           solnInfo.IsPathFound = pathFound;
           solnInfo.ExitFlag = exitFlag;
           solnInfo.NumNodes = numNodes;
           solnInfo.NumIterations = numIterations;
           solnInfo.TreeData = treeData';
           solnInfo.ElapsedTime = elapsedTime;
           solnInfo.PathDistance = pathLength;
       end
       
       function randState = getRandomSample(obj)
           % 生成随机样本（7维状态：x y z qw qx qy qz）
           sampleList = obj.StateSpace.sampleUniform(obj.PopulationSize);
           position = sampleList(:, 1:3);
           orientations = sampleList(:, 4:7);

           % 初始化速度（3维）
           velocity = zeros(obj.PopulationSize, 3);
           goalPos = obj.CurrentGoalState(1:3);

           % 计算成本（基于三维位置，调用 sphere3D）
           cost = costUpdate(position, obj.CostFunction, goalPos);

           % 初始化每个粒子的局部最优
           pBest.Cost = cost;
           pBest.Position = position;

           [minCost, idx] = min(cost);
           gBest.Cost = minCost;
           gBest.Position = position(idx, :);

           % PSO参数
           c1 = obj.LearningCoefficients(1);
           c2 = obj.LearningCoefficients(2);
           w = obj.InertiaWeight;
           wDamping = obj.InertiaDamping;
           iterations = obj.Iterations;

           for i = 1:iterations
               velocity = velocityUpdate(position, velocity, pBest, gBest, w, c1, c2);
               position = position + velocity;
               cost = costUpdate(position, obj.CostFunction, goalPos);
               [gBest, pBest] = updateBest(position, cost, gBest, pBest);
               w = w * wDamping;
           end

           [~, bestIdx] = min(cost);
           randState = [position(bestIdx, :), orientations(bestIdx, :)];

           %% 内部函数
           function newCost = costUpdate(pos, costFunction, goalPos)
               newCost = zeros(size(pos, 1), 1);
               for j = 1:size(pos, 1)
                   % 调用 sphere3D，需要传入 x, y, z 和 goalPos（三维向量）
                   newCost(j) = costFunction(pos(j,1), pos(j,2), pos(j,3), goalPos);
               end
           end

           function newVelocity = velocityUpdate(pos, velocity, pBest, gBest, w, c1, c2)
               newVelocity = zeros(size(velocity));
               for k = 1:size(velocity, 1)
                   newVelocity(k, :) = w * velocity(k, :) + ...
                       c1 * rand * (pBest.Position(k, :) - pos(k, :)) + ...
                       c2 * rand * (gBest.Position - pos(k, :));
               end
           end

           function [gBestOut, pBestOut] = updateBest(pos, cost, gBestIn, pBestIn)
               [minCost, idx] = min(cost);
               if minCost < gBestIn.Cost
                   gBestIn.Cost = minCost;
                   gBestIn.Position = pos(idx, :);
               end
               for l = 1:length(cost)
                   if cost(l) < pBestIn.Cost(l)
                       pBestIn.Cost(l) = cost(l);
                       pBestIn.Position(l, :) = pos(l, :);
                   end
               end
               gBestOut = gBestIn;
               pBestOut = pBestIn;
           end
       end
       
   end
   
end