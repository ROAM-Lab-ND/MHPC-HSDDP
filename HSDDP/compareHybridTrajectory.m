function equal = compareHybridTrajectory(HT1, HT2)
    equal = 1;
    assert(length(HT1)==length(HT2),'Hybrid trajectories have different segments.');
    for idx = 1:length(HT1)
        equal = HT1.compare(HT2);
        if equal == 0
            return
        end
    end
end