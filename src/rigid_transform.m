function poseB = rigid_transform(poseA,rAB_body)

% poseA: Nx7 = [t x y z roll pitch yaw]   (Euler in cols 5:7, order 'XYZ')
% rAB_body: 3x1 offset from A->B in the body/local frame
% poseB: Nx7 same format, with updated xyz and same orientation/time
n = size(poseA,1);
poseB = poseA;
% r = rAB_body(:);

for k = 1:n
    R = eul2rotm(poseA(k,5:7), 'XYZ');
    poseB(k,1:3) = poseA(k,1:3) + (R*rAB_body).';
end
poseB(:,2:4) = poseB(:,2:4) - mean(poseB(:,2:4));
end