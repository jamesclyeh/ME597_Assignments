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
x1 = x0;
time = 10; % time in seconds

l = 0.3;
dynamic_model = [0 1 l; ...
                 -1*cosd(30) -1*sind(30) l; ...
                 cosd(30) -1*sind(30) l];
r = 0.25;
input = [5 5 5];

x = [];
for i=1:time * refresh_rate
    rotational_matrix = [cos(x1(3)) -1 * sin(x1(3)) 0; ...
                         sin(x1(3)) cos(x1(3)) 0; ...
                         0 0 1];
    x1 = x1 + dt * r * inv(dynamic_model * rotational_matrix) * input';
    x(:,i) = x1;
end

% Plot
figure(1); clf; hold on;
plot( x0(1), x0(2), 'bo', 'MarkerSize', 5, 'LineWidth', 3)
plot( x(1,:),x(2,:), 'ro', 'MarkerSize', 3)
title('Motion Model of Distribution Free two-wheeled robot spinning in a circle')
xlabel('x (m)');
ylabel('y (m)');
axis ([-3 ,3 ,-3, 3])

%% move in straight line
x1 = x0;
time = 10; % time in seconds

l = 0.3;
dynamic_model = [0 1 l; ...
                 -1*cosd(30) -1*sind(30) l; ...
                 cosd(30) -1*sind(30) l];
r = 0.25;
input = [0 -1 1];

x = [];
for i=1:time * refresh_rate
    rotational_matrix = [cos(x1(3)) -1 * sin(x1(3)) 0; ...
                         sin(x1(3)) cos(x1(3)) 0; ...
                         0 0 1];
    x1 = x1 + dt * r * inv(dynamic_model * rotational_matrix) * input';
    x(:,i) = x1;
end

% Plot
figure(2); clf; hold on;
plot( x0(1), x0(2), 'bo', 'MarkerSize', 5, 'LineWidth', 3)
plot( x(1,:),x(2,:), 'ro', 'MarkerSize', 3)
title('Motion Model of Distribution Free two-wheeled robot moving in a straight line')
xlabel('x (m)');
ylabel('y (m)');
axis ([-3 ,3 ,-3, 3]);

%% move in circle
x1 = x0;
time = 15; % time in seconds

l = 0.3;
dynamic_model = [0 1 l; ...
                 -1*cosd(30) -1*sind(30) l; ...
                 cosd(30) -1*sind(30) l];
r = 0.25;
input = [-1.5 2.0 1.0];

x = [];
for i=1:time * refresh_rate
    rotational_matrix = [cos(x1(3)) -1 * sin(x1(3)) 0; ...
                         sin(x1(3)) cos(x1(3)) 0; ...
                         0 0 1];
    x1 = x1 + dt * r * inv(dynamic_model * rotational_matrix) * input';
    x(:,i) = x1;
end

% Plot
figure(3); clf; hold on;
plot( x0(1), x0(2), 'bo', 'MarkerSize', 5, 'LineWidth', 3)
plot( x(1,:),x(2,:), 'ro', 'MarkerSize', 3)
title('Motion Model of Distribution Free two-wheeled robot moving in a circle line')
xlabel('x (m)');
ylabel('y (m)');
axis ([-3 ,3 ,-3, 3]);

%% move in circle with 2m diameter
x1 = x0;
time = 15; % time in seconds

l = 0.3;
dynamic_model = [0 1 l; ...
                 -1*cosd(30) -1*sind(30) l; ...
                 cosd(30) -1*sind(30) l];
r = 0.25;

input = ((1 / r) * dynamic_model * [1 0 1]')';

x = [];
for i=1:time * refresh_rate
    rotational_matrix = [cos(x1(3)) -1 * sin(x1(3)) 0; ...
                         sin(x1(3)) cos(x1(3)) 0; ...
                         0 0 1];
    x1 = x1 + dt * r * inv(dynamic_model * rotational_matrix) * input';
    x(:,i) = x1;
end

% Plot
figure(4); clf; hold on;
plot( x0(1), x0(2), 'bo', 'MarkerSize', 5, 'LineWidth', 3)
plot( x(1,:),x(2,:), 'ro', 'MarkerSize', 3)
title('Motion Model of Distribution Free two-wheeled robot moving in a circle line')
xlabel('x (m)');
ylabel('y (m)');
axis equal;

%% model with additive gaussian disturbance
