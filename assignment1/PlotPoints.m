function [] = PlotPoints(x0, predicted_points, actual_points, title_str, axis_param, n)
% PlotPoints - plot wrapper

    figure(n); clf; hold on;
    plot( x0(1), x0(2), 'bo', 'MarkerSize', 5, 'LineWidth', 3);
    plot( predicted_points(1,:), predicted_points(2,:), 'r-', 'MarkerSize', 3);
    plot( actual_points(1,:), actual_points(2,:), 'g-', 'MarkerSize', 3);
    xlabel('x (m)');
    ylabel('y (m)');
    title(title_str);
    axis(axis_param);
end

