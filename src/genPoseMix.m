function trajectory = genPoseMix(n)
    trajectory = genTrajectory('cam', n);
    t2 = genTrajectory('syn', 3* n);
    trajectory(:,4) = t2(1:length(trajectory),4);
    % trajectory(:,2:5) = 0;
    % trajectory(:,7) = 0;
end
