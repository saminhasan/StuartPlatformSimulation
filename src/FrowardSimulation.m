clc; close all; clear all; %#ok<CLALL>
data_paths = {'C:\Users\james\OneDrive\Desktop\CaptureU\UBCO Bra Rig Tests\paired\34-B-003-C1', ...
                'C:\Users\james\OneDrive\Desktop\CaptureU\UBCO Bra Rig Tests\paired\36-B-001', ...
                'C:\Users\james\OneDrive\Desktop\CaptureU\UBCO Bra Rig Tests\paired\34-B-002'};
delay = [9.9, 9.35, 13.62];
k=3;
DATA_PATH = data_paths{k};
xlsxf = fullfile(DATA_PATH, 'motor_data.xlsx');
csvf = fullfile(DATA_PATH, 'imu_data.csv');
T = readtable(csvf, "VariableNamingRule", "preserve");
[act, ref] = extractMotorAngles(xlsxf,1000);
params;
sp = StuartPlatform(r, n, rB, dB, rP, dP);
fcLPF = 50;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% --- run sim with act ---
jointAngles = act;
time = jointAngles(:,1);
tf = time(end); %#ok<NASGU>
simAct = sim("SP.slx");

%% --- run sim with ref ---
jointAngles = ref;
time = jointAngles(:,1);
tf = time(end);
simRef = sim("SP.slx");

%% --- extract + LP filter sim accel ---
tAct = simAct.imu_sim_ax.Time;
tRef = simRef.imu_sim_ax.Time;
imut = T{:,1};

accAct = [ ...
    squeeze(simAct.imu_sim_ax.Data), ...
    squeeze(simAct.imu_sim_ay.Data), ...
    squeeze(simAct.imu_sim_az.Data)];

accRef = [ ...
    squeeze(simRef.imu_sim_ax.Data), ...
    squeeze(simRef.imu_sim_ay.Data), ...
    squeeze(simRef.imu_sim_az.Data)];
imu  = T{:,2:4};

fsAct = 1/mean(diff(tAct));
fsRef = 1/mean(diff(tRef));
fsImu = 1/mean(diff(imut));

accAct = LPFilter(accAct, fsAct, fcLPF);
accRef = LPFilter(accRef, fsRef, fcLPF);
imu   = LPFilter(imu, fsImu, fcLPF);

imuX = imu(:,1); imuY = imu(:,2); imuZ = imu(:,3);
actX = accAct(:,1); actY = accAct(:,2); actZ = accAct(:,3);
refX = accRef(:,1); refY = accRef(:,2); refZ = accRef(:,3);

figure('Name','Accel X');
plot(imut, -(imuZ-mean(imuZ)), ...
     tAct+delay(k), actX-mean(actX), ...
     tRef+delay(k), refX-mean(refX));
grid on
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
title('Acceleration X');
legend('IMU (-Z)','Act Sim X','Ref Sim X','Location','best');

figure('Name','Accel Y');
plot(imut, imuY-mean(imuY), ...
     tAct+delay(k), actY-mean(actY), ...
     tRef+delay(k), refY-mean(refY));
grid on
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
title('Acceleration Y');
legend('IMU Y','Act Sim Y','Ref Sim Y','Location','best');

figure('Name','Accel Z');
plot(imut, imuX-mean(imuX), ...
     tAct+delay(k), actZ-mean(actZ), ...
     tRef+delay(k), refZ-mean(refZ));
grid on
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)');
title('Acceleration Z');
legend('IMU X','Act Sim Z','Ref Sim Z','Location','best');
% 