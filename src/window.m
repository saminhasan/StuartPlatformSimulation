function wtrajectory =  window(trajectory, easeTime, fs)
n_sample = length(trajectory);
win = tukeywin(n_sample, 2*easeTime*fs/n_sample);
wtrajectory = zeros(size(trajectory));
wtrajectory(:,1) = trajectory(:,1);
wtrajectory(:,2:end) = trajectory(:,2:end) .* win;
figure;
plot(trajectory(:,1), win);
end