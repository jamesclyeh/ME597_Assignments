function [predicted, actual] = MoveRobot(prev_predicted, prev_actual, l, r, input, update_rate)
%MOVEROBOT - move robot given input

    dynamic_model = [0 1 l; ...
                     -1*cosd(30) -1*sind(30) l; ...
                     cosd(30) -1*sind(30) l];

    dt = 1 / update_rate;
    
    rotational_matrix_predicted = [cos(prev_predicted(3)) -1 * sin(prev_predicted(3)) 0; ...
                                   sin(prev_predicted(3)) cos(prev_predicted(3)) 0; ...
                                   0 0 1];
    rotational_matrix_actual = [cos(prev_actual(3)) -1 * sin(prev_actual(3)) 0; ...
                                sin(prev_actual(3)) cos(prev_actual(3)) 0; ...
                                0 0 1];
    x1 = prev_actual + dt * r * inv(dynamic_model * rotational_matrix_actual) * input';
    predicted = prev_predicted + dt * r * inv(dynamic_model * rotational_matrix_predicted) * input';
    actual = [x1(1) + normrnd(0, 0.01); x1(2) + normrnd(0, 0.01); x1(3) + normrnd(0, 0.1)]; 
end

