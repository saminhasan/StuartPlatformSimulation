function wtrajectory = window(trajectory, easeTime, fs)
    holdTime = 2.0;
    n = size(trajectory,1);
    win = tukeywin(n, 2*easeTime*fs/n);
    wtrajectory = trajectory;
    x = trajectory(:,2:end);
    mu = mean(x,1);
    wtrajectory(:,2:end) = (x - mu) .* win + mu;
    nHold = round(holdTime * fs);
    holdRows = repmat(wtrajectory(end,:), nHold, 1);
    holdRows(:,1) = wtrajectory(end,1) + (1:nHold)'/fs;
    wtrajectory = [wtrajectory; holdRows];
end
% function wtrajectory =  window(trajectory, easeTime, fs)
%     n_sample = length(trajectory);
%     win = tukeywin(n_sample, 2*easeTime*fs/n_sample);
%     wtrajectory = zeros(size(trajectory));
%     wtrajectory(:,1) = trajectory(:,1);
%     wtrajectory(:,2:end) = trajectory(:,2:end) .* win;
% end