function obj = StuartPlatform(r, n, rB, dB, rP, dP)
    % StuartPlatform class constructor
    % r: radius of the crank arm
    % n: crank arm to connecting rod length ratio
    obj.zshift = 8.5 * 0.0254;
    obj.xshift = - 0.1;
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
    obj.homez = mean(obj.P(:,3));
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
end

function motorAngles = moveFunc(obj, trajectory)
    motorAngles = zeros(size(trajectory));
        for row=1:length(trajectory)
            pose = trajectory(row,2:7);
            t = [pose(1) + obj.xshift, pose(2), pose(3)+ (obj.homez) + (obj.zshift)]';
            R = rotx(rad2deg(pose(4))) * roty(rad2deg(pose(5))) * rotz(rad2deg(pose(6)));
            offset = R * -[obj.xshift; 0.0; obj.zshift];
            t = t + offset;
            % t = [pose(1); pose(2); pose(3) + obj.homez] + R * -[obj.xshift; 0; obj.zshift];
            l = repmat(t, 1, 6)' + (R * obj.Pp')' - obj.B; % leg length
            ek = 2 * obj.r * l(:,3);
            fk = 2 * obj.r * (cos(obj.betaB') .* l(:,1) + sin(obj.betaB') .* l(:,2));
            theta = asin(((vecnorm(l, 2, 2).^2) - ((obj.n^2 - 1)*obj.r^2))./ sqrt(ek.^2 + fk.^2)) - atan2(fk, ek);
            motorAngles(row,2:7) = theta';
        end
     motorAngles(:,1) = trajectory(:,1);
end