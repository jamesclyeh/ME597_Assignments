%====================================================
% Three wheeled omni-directional robot dynamic model
%====================================================

% sym vars init
l = sym('l');
r = sym('r');
w1 = sym('w1');
w2 = sym('w2');
w3 = sym('w3');
theta = sym('theta');

%% disturbance free model
input = [w1 w2 w3]';
rotational_matrix = [cos(theta) -1 * sin(theta) 0; ...
                     sin(theta) cos(theta) 0; ...
                     0 0 1];
dynamic_model = [0 1 l; ...
                 -1*cosd(30) -1*sind(30) l; ...
                 cosd(30) -1*sind(30) l];
output = r * inv(dynamic_model * rotational_matrix) * input;

% state init
x0 = [0 0 0]';
refresh_rate = 10;
dt = 1 / refresh_rate;


%% spinning in circle
rotate_out = subs(output, [l, r, w1, w2, w3, theta], [0.3, 0.25, 5, 5, 5, 0]);
assert(rotate_out(1) == 0, 'X velocity is 0 when rotating');
assert(rotate_out(2) == 0, 'Y velocity is 0 when rotating');

x1 = x0;
time = 15; % time in seconds
for i=1:time * refresh_rate
    dynamic_model = [0 1 0.3; ...
                     -1 * cosd(30) -1 * sind(30) 0.3; ...
                     cosd(30) -1 * sind(30) 0.3];
    rotational_matrix = [cos(x1(3)) -1 * sin(x1(3)) 0; ...
                         sin(x1(3)) cos(x1(3)) 0; ...
                         0 0 1];
    x1 = x1 + dt * 0.25 * inv(dynamic_model*rotational_matrix) * [-1.5 2.0 1.0]'
    x(:,i) = x1;
end

% Plot
figure(1); clf; hold on;
plot( x0(1), x0(2), 'bo', 'MarkerSize', 5, 'LineWidth', 3)
plot( x(1,:),x(2,:), 'ro', 'MarkerSize', 3)
title('Motion Model of Distribution Free two-wheeled robot spinning in a circle')
xlabel('x (m)');
ylabel('y (m)');
axis equal

%% move in straight line
straight_out = subs(output, [l, r, w1, w2, w3, theta], [0.3, 0.25, 0, -1, 1, 0]);
assert(straight_out(3) == 0, 'angular velocity is 0 when moving straight');
assert(straight_out(2) == 0, 'Y velocity is 0 when moving straight');

time = 5; % time in seconds
x1 = x0;
for i=1:time * refresh_rate
    x1 = x1 + dt * straight_out;
    x(:,i) = x1;
end

% Plot
figure(2); clf; hold on;
plot( x0(1), x0(2), 'bo', 'MarkerSize', 5, 'LineWidth', 3)
plot( x(1,:),x(2,:), 'ro', 'MarkerSize', 3)
title('Motion Model of Distribution Free two-wheeled robot moving in a straight line')
xlabel('x (m)');
ylabel('y (m)');
axis equal

%% move in circle
circle_out = subs(output, [l, r, w1, w2, w3, theta], [0.3, 0.25, -1.5, 2.0, 1.0, 0]);
%assert(circle_out(3) == 0, 'angular velocity is 0 when moving straight');
%assert(circle_out(2) == 0, 'Y velocity is 0 when moving straight');

time = 5; % time in seconds
x1 = x0;
for i=1:time * refresh_rate
    x1 = x1 + dt * circle_out;
    x(:,i) = x1;
end

% Plot
figure(3); clf; hold on;
plot( x0(1), x0(2), 'bo', 'MarkerSize', 5, 'LineWidth', 3)
plot( x(1,:),x(2,:), 'ro', 'MarkerSize', 3)
title('Motion Model of Distribution Free two-wheeled robot moving in a straight line')
xlabel('x (m)');
ylabel('y (m)');
axis equal

%% model with additive gaussian disturbance
