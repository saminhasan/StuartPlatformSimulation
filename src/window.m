function wtrajectory =  window(trajectory, easeTime, fs)
n_sample = length(trajectory);
win = tukeywin(n_sample, easeTime*fs/(2*n_sample)/2);
wtrajectory = zeros(size(trajectory));
wtrajectory(:,1) = trajectory(:,1);
wtrajectory(:,2:end) = trajectory(:,2:end) .* win;

end