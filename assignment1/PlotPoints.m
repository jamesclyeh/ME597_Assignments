function [] = PlotPoints(x0, points, title_str, axis_param, n)
% PlotPoints - plot wrapper

    figure(n); clf; hold on;
    plot( x0(1), x0(2), 'bo', 'MarkerSize', 5, 'LineWidth', 3);
    plot( points(1,:), points(2,:), 'ro', 'MarkerSize', 3);
    xlabel('x (m)');
    ylabel('y (m)');
    title(title_str);
    axis(axis_param);
end

