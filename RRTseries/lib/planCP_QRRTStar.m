classdef planCP_QRRTStar < planPSORRTStar & planC_QRRTStar 

    methods
        function obj = planCP_QRRTStar(stateSpace, stateValidator)
            obj@planPSORRTStar(stateSpace, stateValidator)
            obj@planC_QRRTStar(stateSpace, stateValidator)
        end
                
    end
end