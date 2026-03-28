function obj = StuartPlatform(r, n, rB, dB, rP, dP)
    % StuartPlatform class constructor
    % r: radius of the crank arm
    % n: crank arm to connecting rod length ratio
    obj.r = r;
    obj.n = n;
    obj.d = n * r;
    gamma = deg2rad(repelem([0,120,240],2));
    beta  = gamma + repmat([-pi/2, pi/2], 1, 3); %

    obj.gammaB = gamma;
    obj.betaB  = beta;
    obj.gammaP = gamma;
    obj.betaP  = beta;

    obj.B = [rB*cos(gamma) + dB*cos(beta); rB*sin(gamma) + dB*sin(beta); zeros(1,6)]'; % Base points / Servo Arm base
    obj.H = obj.B + [r*cos(beta); r*sin(beta); zeros(1,6)]'; % Servo arm tips
    obj.P = [rP*cos(gamma) + dP*cos(beta); rP*sin(gamma) + dP*sin(beta); zeros(1,6)]'; % platform co-ordinate in B frame
    obj.Pp = obj.P; % in P frame, platform points
    obj.P(:,3) = sqrt(obj.d^2 - sum((obj.H - obj.P).^2, 2));
    obj.homez = [0; 0; mean(obj.P(:,3))'];
    obj.HP = obj.P - obj.H;
    obj.BH = obj.H - obj.B;
    %--------calculate quaternion for correct simscape initialization------------------------------------------
    obj.q_rots = zeros(6, 4);
    for i = 1:6
        u = rotz(rad2deg(beta(i))) * rotx(90) * [1 0 0; 0 1 0; 0 0 1]; % u is the co-ordinate frame at the tip of the servo
        v = u\obj.HP(i,:)';% from o frame to u frame
        obj.q_rots(i, :) = calcQuat([1, 0 ,0]', v); % calculate the quat w.r.t to x frame.
    end
    %--------calculate quaternion------------------------------------------
    obj.move = @(pose) moveFunc(obj, pose);
    obj.calcP = @(pose) calcFunc(obj, pose);
    obj.calcQrots = @(pose, theta) calcQrotsFunc(obj, pose, theta);
end

function motorAngles = moveFunc(obj, trajectory)
    motorAngles = zeros(size(trajectory));
        for row=1:length(trajectory)
            pose = trajectory(row,2:7);
            R = eul2rotm(pose(4:6), 'XYZ');
            t = pose(1:3)' + obj.homez;
            l = repmat(t, 1, 6)' + (R * obj.Pp')' - obj.B; % leg length
            ek = 2 * obj.r * l(:,3);
            fk = 2 * obj.r * (cos(obj.betaB') .* l(:,1) + sin(obj.betaB') .* l(:,2));
            theta = asin(((vecnorm(l, 2, 2).^2) - ((obj.n^2 - 1)*obj.r^2))./ sqrt(ek.^2 + fk.^2)) - atan2(fk, ek);
            motorAngles(row,2:7) = theta';
        end
     motorAngles(:,1) = trajectory(:,1);
end
function q_rots = calcQrotsFunc(obj, pose, theta)
    % pose = [x y z Rx Ry Rz]
    % theta = 6x1 or 1x6 motor angles

    R = eul2rotm(pose(4:6), 'XYZ');
    t = pose(1:3)' + obj.homez;

    % 1) platform points in base frame
    P = repmat(t, 1, 6)' + (R * obj.Pp')';

    % 2) horn tip points in base frame
    theta = theta(:);
    H = obj.B + [ ...
        obj.r*cos(theta).*cos(obj.betaB(:)), ...
        obj.r*cos(theta).*sin(obj.betaB(:)), ...
        obj.r*sin(theta) ...
    ];

    % 3) rod vectors in base frame
    HP = P - H;

    % 4) same quaternion process as constructor
    q_rots = zeros(6,4);
    for i = 1:6
        u = roty(rad2deg(theta(i))) * rotz(rad2deg(obj.betaB(i))) * rotx(90);
        v = u \ HP(i,:)';
        q_rots(i,:) = calcQuat([1;0;0], v);
    end
end