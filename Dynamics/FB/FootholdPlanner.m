classdef FootholdPlanner < handle
    properties
        model
        PhaseSeq
        gait
        time
        vd
    end
    methods 
        function Planner = FootholdPlanner(model, gait, currentPhase, n_Phases, time, vd)
            if nargin > 1
                Initialization(Planner,model,gait,currentPhase,n_Phases, time, vd);
            end
        end
        
        function Initialization(Planner, model, gait, currentPhase, n_Phases, time, vd)
            Planner.model = model;
            Planner.gait = gait;
            Planner.time = time;
            Planner.vd = vd;
            Planner.PhaseSeq = gait.get_gaitSeq(currentPhase, n_Phases);
        end
        
        function Foothold = getFoothold(Planner, x0)
            Foothold = zeros(4, length(Planner.PhaseSeq));
            xCoM = x0(1);
            zCoM = x0(2);
            for idx = 1:length(Planner.PhaseSeq)
                if idx > 1
                    xCoM = xCoM + Planner.vd*(Planner.time(idx-1)-Planner.model.dt);
                end
                pCoM = [xCoM, zCoM]';
                offset = Planner.vd*Planner.time(idx)/2;
                if strcmp(Planner.gait.name, 'bounding')
                    switch Planner.PhaseSeq(idx)
                        case 1
%                             pHip = Planner.model.getPosition([pCoM;zeros(5,1)],4,zeros(2,1));
%                             Foothold_candidate = [zeros(2,1); pHip(1)+Planner.vd*Planner.time(idx)/2; -0.404]; 
                              Foothold_candidate = [zeros(2,1);
                                                    xCoM + Planner.model.hipLoc{2}(1)+offset; -0.404];
                        case 3
%                             pHip = Planner.model.getPosition([pCoM;zeros(5,1)],2,zeros(2,1));
%                             Foothold_candidate = [pHip(1)+Planner.vd*Planner.time(idx)/2; -0.404; zeros(2,1)];  
                              Foothold_candidate = [xCoM + Planner.model.hipLoc{1}(1)+offset; -0.404;
                                                    zeros(2,1);];
                        case {2,4}
                            Foothold_candidate = zeros(4,1);
                    end
                end
                Foothold(:, idx) = Foothold_candidate;
            end
        end
    end
end