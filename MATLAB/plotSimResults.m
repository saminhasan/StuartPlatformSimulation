function plotSimResults(out, trajectory, motorAngles, sp)
    params
    torque_calc(out, N, rated_torque, peak_torque);
    motion_comp(out, trajectory, sp.homez);
end