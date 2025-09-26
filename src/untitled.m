fg = openfig('wj.fig');
set(fg,'Color','w');
ax = findall(fg,'Type','axes');
if numel(ax)~=1
    % If you have multiple axes, pick the one you want
    ax = ax(1);
end

% Light theme axes
set(ax,'Color','w','XColor','k','YColor','k');
xlabel(ax,'Time (s)');
ylabel(ax,'Force (N)');
grid(ax,'on'); grid(ax,'minor');

% Remove any existing legend
delete(findall(fg,'Type','Legend'));

% Get all line objects, in plot order (flip since findall returns reverse)
allLines = flipud(findall(ax,'Type','line'));
nLines   = numel(allLines);

% Target indices 1,4,7,10,13,16 (cap at number of lines)
idx = 1:3:nLines;
idx = idx(1:min(6,numel(idx)));  % keep up to 6 entries

% Hide *all* real lines from the legend
for h = reshape(allLines,1,[])
    set(get(get(h,'Annotation'),'LegendInformation'), 'IconDisplayStyle','off');
end

% Build dummy legend items, copying style from the selected lines
hold(ax,'on');
dummy = gobjects(0);
for k = 1:numel(idx)
    src = allLines(idx(k));
    % Copy key visual props; fall back safely if missing
    try Lw = get(src,'LineWidth'); catch, Lw = 1.5; end
    try Ls = get(src,'LineStyle'); catch, Ls = '-'; end
    try Mk = get(src,'Marker');    catch, Mk = 'none'; end
    try Cl = get(src,'Color');     catch, Cl = [0 0 0]; end

    % Create a dummy line that doesn't render data but shows in legend
    dummy(k) = plot(ax, NaN, NaN, ...
        'LineWidth', Lw, 'LineStyle', Ls, 'Marker', Mk, 'Color', Cl, ...
        'DisplayName', sprintf('Rod-%d', k));
end

% Add the legend using the dummy handles
leg = legend(ax, dummy, 'Location','best');
set(leg,'TextColor','k','EdgeColor',[0.85 0.85 0.85]); % light theme legend box

% Optional: keep legend from auto-updating if you modify plots later
set(leg,'AutoUpdate','off');

grid on
grid minor
