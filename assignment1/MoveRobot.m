function [predicted, actual, kalman] = MoveRobot(prev_predicted, prev_actual, l, r, input, update_rate)
%MOVEROBOT - move robot given input

    dynamic_model = [0 1 l; ...
                     -1*cosd(30) -1*sind(30) l; ...
                     cosd(30) -1*sind(30) l];

    dt = 1 / update_rate;
    
    rotational_matrix_predicted = [cos(prev_predicted(3)+pi/2) sin(prev_predicted(3)+pi/2) 0; ...
                                   -1 * sin(prev_predicted(3)+pi/2) cos(prev_predicted(3)+pi/2) 0; ...
                                   0 0 1];
    rotational_matrix_actual = [cos(prev_actual(3)+pi/2) sin(prev_actual(3)+pi/2) 0; ...
                                -1 * sin(prev_actual(3)+pi/2) cos(prev_actual(3)+pi/2) 0; ...
                                0 0 1];
    actual_velocity = r * inv(dynamic_model * rotational_matrix_actual) * input';
    x1 = prev_actual + dt *  actual_velocity;
    latest_predicted = prev_predicted + dt * r * inv(dynamic_model * rotational_matrix_predicted) * input';
    latest_actual = x1 + [normrnd(0, 0.01); normrnd(0, 0.01); normrnd(0, 0.1*pi()/180)];
    
    predicted = latest_predicted;
    actual = latest_actual;
end

