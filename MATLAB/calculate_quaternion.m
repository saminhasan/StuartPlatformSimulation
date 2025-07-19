function quaternion = calculate_quaternion(U, bp)
    cross_product = cross(U, bp);
    angle_radians = acos(dot(U, bp) / (norm(U) * norm(bp)));
    axis_of_rotation = cross_product / norm(cross_product);
    qw = cos(angle_radians / 2);
    qx = axis_of_rotation(1) * sin(angle_radians / 2);
    qy = axis_of_rotation(2) * sin(angle_radians / 2);
    qz = axis_of_rotation(3) * sin(angle_radians / 2);
    quaternion = [qw, qx, qy, qz];
    quaternion = quaternion / norm(quaternion);
end