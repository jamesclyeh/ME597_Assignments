function x1 = MoveRobot(x0, l, r, input, update_rate)
%MOVEROBOT - move robot given input

    dynamic_model = [0 1 l; ...
                     -1*cosd(30) -1*sind(30) l; ...
                     cosd(30) -1*sind(30) l];

    dt = 1 / update_rate;
    x1 = x0;
    
    rotational_matrix = [cos(x1(3)) -1 * sin(x1(3)) 0; ...
                         sin(x1(3)) cos(x1(3)) 0; ...
                         0 0 1];
    x1 = x1 + dt * r * inv(dynamic_model * rotational_matrix) * input';
end

