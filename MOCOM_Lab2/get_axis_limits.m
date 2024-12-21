function [xlim, ylim, zlim] = get_axis_limits(config_matrix)
    
    % retrive the single axis value along all the steps of the movement
    Xvalue = config_matrix(1, :, :);
    Yvalue = config_matrix(2, :, :);
    Zvalue = config_matrix(3, :, :);
    
    % find the bound of those value
    [xmin, xmax] = bounds(Xvalue, "all");
    [ymin, ymax] = bounds(Yvalue, "all");
    [zmin, zmax] = bounds(Zvalue, "all");

    % the axis(axislimits) function cannot work with equal value, make sure
    % to break ties
    if xmin == xmax
        xmax = xmax+1;
    end

    if ymin == ymax
        ymax = ymax+1;
    end

    if zmin == ymax
        zmax = zmax+1;
    end
   
    % return the boundaries of the plot
    xlim = [xmin, xmax];
    ylim = [ymin, ymax];
    zlim = [zmin, zmax];

end