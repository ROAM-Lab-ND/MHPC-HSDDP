classdef Gait < handle
    properties
        name
        gait
        bounding_gait = [1,2,3,4]; % one bounding gait cycle with unique phase index
        
        gait_Enabled
        
        period_time
        timeStance
        timeSwing
        timeStanceRemaining
        timeSwingRemaning
        
        switchingPhase
        phaseVariable
        phaseStance
        phaseSwing
        phaseScale
        phaseOffset
        
        contactStateScheduled
        contactStatePrev
        touchdownScheduled
        liftoffScheduled
    end
    
    methods
        function G = Gait(name)
            G.name = name;
            if strcmp(name, 'bounding')
                G.gait = G.bounding_gait;
            end
        end
        
        function phaseSeq = get_gaitSeq(G, currentPhase, n_Phases)
            phaseSeq = zeros(1,n_Phases);
            phaseSeq(1) = currentPhase;
            for i = 2:n_Phases
                phaseSeq(i) = G.get_nextPhase(phaseSeq(i-1));
            end
        end
        
        function nextPhase = get_nextPhase(G, currentPhase)
            currentIdx = find(G.gait == currentPhase);
            if currentIdx == length(G.gait)
                nextPhase = G.gait(1);
            else
                nextPhase = G.gait(currentIdx+1);
            end
        end
        
        function define_your_gait(G, yourGait)
            C = unique(yourGait);
            if length(C) ~= length(yourGait)
                error('Phase index has to be unique in one gait cycle.')
            end
            G.gait = yourGait;
        end
    end
end