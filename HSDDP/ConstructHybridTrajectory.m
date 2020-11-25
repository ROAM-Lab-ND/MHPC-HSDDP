function hybridT = ConstructHybridTrajectory(Phases, N_horizons)
    n_Phases = length(Phases);
    assert(n_Phases == length(N_horizons), 'Fail to construct hybrid trajectory. Dimensions do not match!');
    
    % preallocate to save memory
    hybridT = repmat(PhaseTrajectory(Phases(1).xsize, Phases(1).usize, Phases(1).ysize, N_horizons(1)),[1, n_Phases]);
    
    % Initialize the remainig trajectory
    for idx = 2:length(n_Phases)
        hybridT(idx) = PhaseTrajectory(Phases(idx).xsize, Phases(idx).usize, Phases(idx).ysize, N_horizons(idx));
    end
end