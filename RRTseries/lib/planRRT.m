% filepath: /f:/机械臂/小论文/小论文代码/RRTseries_dev/lib/planRRT.m
classdef planRRT < handle
    
    properties (Constant, Access = protected)
        GoalReached = 1
        MaxIterationsReached = 2
        MaxNumTreeNodesReached = 3
        MotionInCollision = 4
        InProgress = 5
        GoalReachedButContinue = 6
        ExistingState = 7
        Unassigned = 8
        MaxTimeElapsed = 50;
    end
    
    properties
        StateSpace
        StateValidator
        MaxNumTreeNodes
        MaxIterations
        MaxConnectionDistance
        GoalBias
        GoalReachedFcn    % 请设定为支持 3D 判断的函数
        ContinueAfterGoalReached
        MaxTime
    end    
    
    properties (Access = protected)
        CurrentGoalState
    end
    
    methods
        
        function obj = planRRT(stateSpace, stateValidator)
            obj.StateSpace = stateSpace;
            obj.StateValidator = stateValidator;
            obj.MaxNumTreeNodes = 1e4;
            obj.MaxIterations = 1e4;
            obj.MaxConnectionDistance = 30;   % 根据3D情况调整
            obj.GoalBias = 0.05;
            % 使用支持 3D 状态判断的目标检测函数，
            % 此函数应只比较状态的前三个分量 [x y z]
            obj.GoalReachedFcn = @checkIfGoalIsReached;  
            obj.ContinueAfterGoalReached = true;
        end
        
        function [pathObj, solnInfo] = plan(obj, start, goal)
            tic
            obj.CurrentGoalState = goal;
            tentativeGoalIds = [];
            
            % 创建搜索树
            treeInternal = nav.algs.internal.SearchTree(start, obj.MaxNumTreeNodes);
            
            % 在最初检测：若起点已满足目标条件，则直接构造路径
            if obj.GoalReachedFcn(obj, start, goal)
                pathObj = navPath(obj.StateSpace, [start; goal]);
                solnInfo = struct();
                solnInfo.IsPathFound = true;
                solnInfo.ExitFlag = obj.GoalReached;
                solnInfo.NumNodes = 1;
                solnInfo.NumIterations = 0;
                solnInfo.TreeData = [start; ...
                                     nan(1, obj.StateSpace.NumStateVariables); ...
                                     start; ...
                                     goal; ...
                                     nan(1, obj.StateSpace.NumStateVariables)];
                solnInfo.ElapsedTime = toc;
                solnInfo.PathDistance = obj.StateSpace.distance(start, goal);
                return;
            end
            
            pathFound = false;
            statusCode = obj.Unassigned;
            numIterations = 0;
            
            for k = 1:obj.MaxIterations
                randState = getRandomSample(obj);  % 从 SE3 状态空间均匀抽样
                if rand() < obj.GoalBias
                    randState = goal;  % 一定概率直接选取目标状态
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
            if statusCode > obj.MotionInCollision
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
            solnInfo.ElapsedTime = toc;
            solnInfo.PathDistance = pathLength;
        end
        
        function [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal)
            statusCode = obj.InProgress;
            newNodeId = nan;
            idx = treeInternal.nearestNeighbor(randState);
            nearestNode = treeInternal.getNodeState(idx);
            
            newState = randState;
            d = obj.StateSpace.distance(randState, nearestNode);
            if d > obj.MaxConnectionDistance
                newState = obj.StateSpace.interpolate(nearestNode, randState, obj.MaxConnectionDistance/d);
            end
            
            if ~obj.StateValidator.isMotionValid(nearestNode, newState)
                statusCode = obj.MotionInCollision;
                return;
            end
            
            newNodeId = treeInternal.insertNode(newState, idx);
            
            % 使用只比较位置的目标检测（支持 SE3）
            if obj.GoalReachedFcn(obj, obj.CurrentGoalState, newState)
                statusCode = obj.GoalReached;
                return;
            end
            
            if newNodeId == obj.MaxNumTreeNodes
                statusCode = obj.MaxNumTreeNodesReached;
                return;
            end

            % 再次检查目标状态（符合 ContinueAfterGoalReached 时）
            if obj.GoalReachedFcn(obj, obj.CurrentGoalState, newState)
                if obj.ContinueAfterGoalReached
                    statusCode = obj.GoalReachedButContinue;
                else
                    statusCode = obj.GoalReached;
                end
                return;
            end
        end
        
        function randState = getRandomSample(obj)
            randState = obj.StateSpace.sampleUniform();
        end
    end
end