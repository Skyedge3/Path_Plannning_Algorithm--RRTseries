% filepath: /f:/机械臂/小论文/小论文代码/RRTseries_dev/lib/planQRRTStar.m
classdef planQRRTStar < planRRTStar

    properties
        ParentDepthSearch
        ParentDepthRewire
    end
    properties(Access = protected)
        ParentNodeIds
    end
    
    methods
        function obj = planQRRTStar(stateSpace, stateValidator)
            obj@planRRTStar(stateSpace, stateValidator);
            obj.ParentDepthSearch = 2;
            obj.ParentDepthRewire = 1;
            obj.ParentNodeIds = containers.Map('KeyType', 'double', 'ValueType', 'double');
        end
        
        function [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal)
            statusCode = obj.InProgress;
            newNodeId = nan;
            
            % 查找离随机样本最近的节点（所有距离计算均基于状态空间，内部只考虑位置）
            idx = treeInternal.nearestNeighbor(randState);
            nearestNode = treeInternal.getNodeState(idx);
            costNN = treeInternal.getNodeCostFromRoot(idx);
            
            d = obj.StateSpace.distance(nearestNode, randState);
            if d < 1e-10
                statusCode = obj.ExistingState;
                return;
            end
            
            newState = randState;
            if d > obj.MaxConnectionDistance
                newState = obj.StateSpace.interpolate(nearestNode, newState, obj.MaxConnectionDistance/d);
                d = obj.MaxConnectionDistance;
            end
            
            costNew = costNN + d;
            
            if ~obj.StateValidator.isMotionValid(nearestNode, newState)
                statusCode = obj.MotionInCollision;
                return;
            end
            
            nearIndices = treeInternal.near(newState);
            costMin = costNew;
            idMin = -1;
            
            % 查找一个代价最小的父节点，注意所有计算均调用 stateSpace.distance（仅位置部分有效）
            for j = 1:length(nearIndices)
                idNear = nearIndices(j);
                stateNear = treeInternal.getNodeState(idNear);
                costNear = treeInternal.getNodeCostFromRoot(idNear);

                costTentative = costNear + obj.StateSpace.distance(stateNear, newState);
                if costMin > costTentative && obj.StateValidator.isMotionValid(stateNear, newState)
                    costMin = costTentative;
                    idMin = idNear;
                end
            end
            
            if idMin >= 0
                nearIndices = nearIndices(nearIndices ~= idMin);
            else
                idMin = idx;
            end
            
            % 深度搜索父节点，确保父节点选择符合3D规划中更优的连接（依旧基于位置距离）
            for n = 1:obj.ParentDepthSearch
                if obj.ParentNodeIds.isKey(idMin)
                    idParent = obj.ParentNodeIds(idMin);
                    stateParent = treeInternal.getNodeState(idParent);
                    costParent = treeInternal.getNodeCostFromRoot(idParent);
                    
                    costTentativeParent = costParent + obj.StateSpace.distance(stateParent, newState);
                    if costMin > costTentativeParent && obj.StateValidator.isMotionValid(stateParent, newState)
                        costMin = costTentativeParent;
                        idMin = idParent;
                    end
                end
            end
     
            % 插入新节点，并记录父子关系
            idNew = treeInternal.insertNode(newState, idMin);
            obj.ParentNodeIds(idNew) = idMin;
            newNodeId = idNew;
            
            % 对附近节点进行重连（rewire），依然调用 stateSpace.distance，
            % 并更新父节点关系，注意所有几何计算仅考虑位移部分（前三元素）
            for k = 1:length(nearIndices)
                idNear = nearIndices(k);
                stateNear = treeInternal.getNodeState(idNear);
                costNear = treeInternal.getNodeCostFromRoot(idNear);
                
                if costNear > costNew + obj.StateSpace.distance(stateNear, newState) && ...
                        obj.StateValidator.isMotionValid(stateNear, newState)
                    rewireStatus = treeInternal.rewire(idNear, newNodeId);
                    obj.ParentNodeIds(idNear) = newNodeId;
                end
            end

            % 使用仅比较位置部分的目标检测函数判断是否到达目标
            if obj.GoalReachedFcn(obj, obj.CurrentGoalState, newState)
                if obj.ContinueAfterGoalReached
                    statusCode = obj.GoalReachedButContinue;
                else
                    statusCode = obj.GoalReached;
                end
                return;
            end
            
            if newNodeId == obj.MaxNumTreeNodes
                statusCode = obj.MaxNumTreeNodesReached;
                return;
            end
        end
    end
    
end