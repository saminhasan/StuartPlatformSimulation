function [trajectory, tf, ts] = genTrajectory(mode, num_cycle)
    switch lower(mode)
        case 'sin'
            [trajectory, tf, ts] = genPoseSine(num_cycle);
            
        case 'syn'
            [trajectory, tf, ts] = genPose(num_cycle);
            
        case 'cam'
            [trajectory, tf, ts] = genaratePoseTK(num_cycle);
            
        case 'imu'
            [trajectory, tf, ts] = genaratePoseImu(num_cycle);
            
        otherwise
            error('genTrajectory:UnknownMode', ...
                  'Unknown mode "%s". Valid modes are sin, syn, cam, imu.', ...
                  mode);
    end
end