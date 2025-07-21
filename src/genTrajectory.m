function trajectory = genTrajectory(type, num_cycle)
    switch lower(type)
        case 'sin'
            trajectory = genPoseSine(num_cycle);
            
        case 'syn'
            trajectory = genPoseSynthetic(num_cycle);
            
        case 'cam'
            trajectory = genPoseCam(num_cycle);
            
        case 'imu'
            trajectory = genPoseImu(num_cycle);
        otherwise
            error('genTrajectory:UnknownType', 'Unknown mode "%s". Valid modes are sin, syn, cam, imu.', type);
    end
end