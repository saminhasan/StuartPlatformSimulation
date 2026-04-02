function [act, ref] = extractMotorAngles(xlsx, fs)
nAxis = 6;

tCell  = cell(1, nAxis);
thCell = cell(1, nAxis);

tStart = -inf;
tEnd   = inf;

% -----------------------------------------
% Build actual motor angles from Axis_1..6
% -----------------------------------------
for ax = 1:nAxis
    sheet = sprintf('Axis_%d', ax);
    T = readtable(xlsx, 'Sheet', sheet);

    t  = double(T.timestamp_us) * 1e-6;
    th = double(T.theta);

    good_t = isfinite(t);
    t  = t(good_t);
    th = th(good_t);

    if isempty(t)
        error('Axis %d has no valid timestamps.', ax);
    end

    [t, order] = sort(t);
    th = th(order);

    [t, lastIdx] = unique(t, 'last');
    th = th(lastIdx);

    good_th = isfinite(th);
    if ~any(good_th)
        error('Axis %d has no valid theta values.', ax);
    end

    i0 = find(good_th, 1, 'first');
    i1 = find(good_th, 1, 'last');

    if t(i0) > tStart, tStart = t(i0); end
    if t(i1) < tEnd,   tEnd   = t(i1); end

    tCell{ax}  = t;
    thCell{ax} = th;
end

if ~(tEnd > tStart)
    error('No common valid time interval across all axes.');
end

for ax = 1:nAxis
    t  = tCell{ax};
    th = thCell{ax};

    in = (t >= tStart) & (t <= tEnd);
    t  = t(in);
    th = th(in);

    if numel(t) < 2
        error('Axis %d has too little data after trimming.', ax);
    end

    th = fillMissing1D(th, sprintf('Axis %d', ax));

    tCell{ax}  = t;
    thCell{ax} = th;
end

tq_abs = (tStart : 1/fs : tEnd).';
nq = numel(tq_abs);

act = zeros(nq, 7);
act(:,1) = tq_abs - tq_abs(1);

for ax = 1:nAxis
    act(:,ax+1) = interp1(tCell{ax}, thCell{ax}, tq_abs, 'spline');
end

if any(isnan(act(:)))
    error('Output act still contains NaNs.');
end

% -----------------------------------------
% Build ref from jog sheet using same time vector
% -----------------------------------------
Tj = readtable(xlsx, 'Sheet', 'jog');

ref = zeros(nq, 7);
ref(:,1) = act(:,1);

for ax = 1:nAxis
    vname = sprintf('j%d', ax);

    if ~ismember(vname, Tj.Properties.VariableNames)
        error('Sheet "jog" is missing column "%s".', vname);
    end

    x = double(Tj.(vname));
    x = x(:);

    if isempty(x)
        error('%s is empty.', vname);
    end

    nx = numel(x);
    if nx < nq
        x(nx+1:nq,1) = NaN;
    else
        x = x(1:nq);
    end

    x = fillMissing1D(x, vname);
    ref(:,ax+1) = x;
end

if any(isnan(ref(:)))
    error('Output ref still contains NaNs.');
end
end


function x = fillMissing1D(x, name)
x = x(:);

nanIdx = isnan(x);
while any(nanIdx)
    x_old = x;

    for k = find(nanIdx).'
        ip  = find(~isnan(x(1:k-1)), 1, 'last');
        inx = k + find(~isnan(x(k+1:end)), 1, 'first');

        hasPrev = ~isempty(ip);
        hasNext = ~isempty(inx);

        if hasPrev && hasNext
            x(k) = 0.5 * (x(ip) + x(inx));
        elseif hasPrev
            x(k) = x(ip);
        elseif hasNext
            x(k) = x(inx);
        end
    end

    if isequaln(x, x_old)
        break;
    end

    nanIdx = isnan(x);
end

if any(isnan(x))
    firstGood = find(~isnan(x), 1, 'first');
    lastGood  = find(~isnan(x), 1, 'last');

    if isempty(firstGood)
        error('%s is all NaN.', name);
    end

    x(1:firstGood-1) = x(firstGood);
    x(lastGood+1:end) = x(lastGood);

    for k = 2:numel(x)-1
        if isnan(x(k))
            x(k) = 0.5 * (x(k-1) + x(k+1));
        end
    end
end

if any(isnan(x))
    error('%s still contains NaNs after fill.', name);
end
end