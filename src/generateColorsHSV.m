function colors = generateColorsHSV(n)
    % Generate N distinct colors using HSV colormap
    hues = linspace(0, 1, n + 1);  % add +1 to avoid repeating first color at end
    hues(end) = [];               % remove last hue to keep only N values
    colors = hsv2rgb([hues(:), ones(n,1), ones(n,1)]); % HSV to RGB
end