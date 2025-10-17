function trajectory = genPoseMix(n)
    trajectory = genTrajectory('cam', n);
    t2 = genPoseSynthetic( 3* n);
    trajectory(:,4) = t2(1:length(trajectory),4);
end
