% filepath: /f:/机械臂/小论文/小论文代码/RRTseries_dev/lib/optQRRTStar.m
classdef planC_QRRTStar < planRRTStar

    properties
        ParentDepthSearch
        ParentDepthRewire
        MaxDeflectionAngle % 最大允许偏转角（弧度）
    end
    properties(Access = protected)
        ParentNodeIds
    end
    properties (Constant)
        MaxDeflectionAngleViolated = 9; % 原状态代码（本例中不再用于返回违规）
    end

    methods
        function obj = optQRRTStar(stateSpace, stateValidator, maxDeflectionAngle)
            obj@planRRTStar(stateSpace, stateValidator);
            
            obj.ParentDepthSearch = 2;
            obj.ParentDepthRewire = 1;
            obj.ParentNodeIds = containers.Map('KeyType', 'double', 'ValueType', 'double');
            if nargin >= 3
                obj.MaxDeflectionAngle = maxDeflectionAngle;
            else
                obj.MaxDeflectionAngle = (33.53)*pi/180; % 最大偏转角设置为33.53°
            end
        end
        
        function [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal)
            statusCode = obj.InProgress;
            newNodeId = nan;
            
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
            
            % 检查父节点与当前节点形成的向量与候选扩展分量之间的转角约束，
            % 若超过约束，则调整扩展方向
            if obj.ParentNodeIds.isKey(idx)
                parentId = obj.ParentNodeIds(idx);
                parentNode = treeInternal.getNodeState(parentId);
                if ~obj.isDeflectionAngleValid(parentNode, nearestNode, newState)
                    % 调整 newState，使得转角恰好为 MaxDeflectionAngle
                    newPos = obj.adjustState(parentNode, nearestNode, newState);
                    newState(1:3) = newPos; % 修改扩展状态的位置部分
                    % 重新计算距离及累计代价
                    d = obj.StateSpace.distance(nearestNode, newState);
                    costNew = costNN + d;
                end
            end
            
            nearIndices = treeInternal.near(newState);
            costMin = costNew;
            idMin = -1;
            
            for j = 1:length(nearIndices)
                idNear = nearIndices(j);
                stateNear = treeInternal.getNodeState(idNear);
                costNear = treeInternal.getNodeCostFromRoot(idNear);

                costTentative = costNear + obj.StateSpace.distance(stateNear, newState);
                if costMin > costTentative && obj.StateValidator.isMotionValid(stateNear, newState) ...
                        && obj.isDeflectionAngleValid(stateNear, nearestNode, newState)
                    costMin = costTentative;
                    idMin = idNear;
                end
            end
            
            if idMin >= 0
                nearIndices = nearIndices(nearIndices ~= idMin);
            else
                idMin = idx;
            end
            
            for n = 1:obj.ParentDepthSearch
                if obj.ParentNodeIds.isKey(idMin)
                    idParent = obj.ParentNodeIds(idMin);
                    stateParent = treeInternal.getNodeState(idParent);
                    costParent = treeInternal.getNodeCostFromRoot(idParent);
                    
                    costTentativeParent = costParent + obj.StateSpace.distance(stateParent, newState);
                    if costMin > costTentativeParent && obj.StateValidator.isMotionValid(stateParent, newState) ...
                            && obj.isDeflectionAngleValid(stateParent, nearestNode, newState)
                        costMin = costTentativeParent;
                        idMin = idParent;
                    end
                end
            end
            
            idNew = treeInternal.insertNode(newState, idMin);
            obj.ParentNodeIds(idNew) = idMin;
            
            newNodeId = idNew;
            
            for k = 1:length(nearIndices)
                idNear = nearIndices(k);
                stateNear = treeInternal.getNodeState(idNear);
                costNear = treeInternal.getNodeCostFromRoot(idNear);
                
                if costNear > costNew + obj.StateSpace.distance(stateNear, newState) ...
                        && obj.StateValidator.isMotionValid(stateNear, newState) ...
                        && obj.isDeflectionAngleValid(stateNear, nearestNode, newState)
                    
                    rewireStatus = treeInternal.rewire(idNear, newNodeId);
                    obj.ParentNodeIds(idNear) = newNodeId;
                end
            end
            
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
        
        % 检查转角是否满足允许范围（仅使用状态的前三个分量，即位置）
        function valid = isDeflectionAngleValid(obj, parent, current, newPoint)
            v1 = current(1:3) - parent(1:3);    % 向量 AB
            v2 = newPoint(1:3) - current(1:3);    % 向量 BC
            
            if norm(v1) < 1e-6 || norm(v2) < 1e-6
                valid = true;
                return;
            end
            
            % 计算夹角（弧度）
            angle = acos( dot(v1, v2) / (norm(v1) * norm(v2)) );
            valid = angle <= obj.MaxDeflectionAngle;
        end
        
        % 调整扩展方向：若候选状态 C（newPoint）与 AB 向量的转角超过最大允许角，
        % 则将扩展方向修改为在平面内旋转 MaxDeflectionAngle 后的方向，
        % 返回调好的位置向量（仅前三个分量）
        function newPos = adjustState(obj, parent, current, candidate)
            % 计算向量 AB 与 BC
            vAB = current(1:3) - parent(1:3);
            vBC = candidate(1:3) - current(1:3);
            dCandidate = norm(vBC);
            
            vAB_norm = vAB / norm(vAB);
            
            % 计算当前转角
            angleCandidate = acos( dot(vAB_norm, vBC/norm(vBC)) );
            
            % 若转角已不超过最大约束，则直接返回候选位置
            if angleCandidate <= obj.MaxDeflectionAngle
                newPos = candidate(1:3);
                return;
            end
            
            % 计算旋转轴（平面为 vAB 与 vBC 所在平面）
            rotAxis = cross(vAB_norm, vBC);
            if norm(rotAxis) < 1e-6
                % 当向量共线时，直接返回原候选位置
                newPos = candidate(1:3);
                return;
            end
            rotAxis = rotAxis / norm(rotAxis);
            
            % 利用 Rodrigues 公式旋转 vAB_norm，旋转角为 MaxDeflectionAngle
            theta = obj.MaxDeflectionAngle;
            newDir = vAB_norm*cos(theta) + cross(rotAxis, vAB_norm)*sin(theta) + ...
                     rotAxis * (dot(rotAxis, vAB_norm))*(1 - cos(theta));
            
            % 新位置为当前节点加上与原来候选扩展长度相同的新方向
            newPos = current(1:3) + dCandidate * newDir;
        end
    end
end