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

%% state init

x0 = [0 0 0]';
refresh_rate = 10;
time = 10;
l = 0.3;
r = 0.25;
axis = [-4 ,4 ,-4, 4];

%% spinning in circle

input = [5 5 5];

predicted_points = [];
actual_points = [];
predicted = x0;
actual = x0;
for i=1:time * refresh_rate
    [predicted, actual] = MoveRobot(predicted, actual, l, r, input, refresh_rate);
    predicted_points(:,i) = predicted;
    actual_points(:,i) = actual;
end

% Plot
title = ('Motion Model of three-wheeled robot spinning in a circle');
PlotPoints(x0, predicted_points, actual_points, title, axis, 1);

%% move in straight line

input = [0 -1 1];

predicted_points = [];
actual_points = [];
predicted = x0;
actual = x0;
for i=1:time * refresh_rate
    [predicted, actual] = MoveRobot(predicted, actual, l, r, input, refresh_rate);
    predicted_points(:,i) = predicted;
    actual_points(:,i) = actual;
end

% Plot
title = 'Motion Model of three-wheeled robot moving in a straight line';
PlotPoints(x0, predicted_points, actual_points, title, axis, 2);

%% move in circle

input = [-1.5 2.0 1.0];

predicted_points = [];
actual_points = [];
predicted = x0;
actual = x0;
for i=1:15 * refresh_rate
    [predicted, actual] = MoveRobot(predicted, actual, l, r, input, refresh_rate);
    predicted_points(:,i) = predicted;
    actual_points(:,i) = actual;
end

% Plot
title = 'Motion Model of three-wheeled robot moving in a circle';
PlotPoints(x0, predicted_points, actual_points, title, 'equal', 3);

%% move in circle with 2m diameter

dynamic_model = [0 1 l; ...
                 -1*cosd(30) -1*sind(30) l; ...
                 cosd(30) -1*sind(30) l];
input = ((1 / r) * dynamic_model * [1 0 1]')';

predicted_points = [];
actual_points = [];
predicted = x0;
actual = x0;
for i=1:7 * refresh_rate
    [predicted, actual] = MoveRobot(predicted, actual, l, r, input, refresh_rate);
    predicted_points(:,i) = predicted;
    actual_points(:,i) = actual;
end

% Plot
title = 'Motion Model of three-wheeled robot moving in a circle';
PlotPoints(x0, predicted_points, actual_points, title, 'equal', 4);

%% move in spiral

dynamic_model = [0 1 l; ...
                 -1*cosd(30) -1*sind(30) l; ...
                 cosd(30) -1*sind(30) l];

predicted_points = [];
actual_points = [];
predicted = x0;
actual = x0;
for i=1:20 * refresh_rate
    input = [(6)/5; (6)/5 - 2*3^(1/2)*i/10; (6)/5 + 2*3^(1/2)*i/10]';
    [predicted, actual] = MoveRobot(predicted, actual, l, r, input, refresh_rate);
    predicted_points(:,i) = predicted;
    actual_points(:,i) = actual;
end

% Plot
title = 'Motion Model of three-wheeled robot moving in a spiral';
PlotPoints(x0, predicted_points, actual_points, [], title, 'equal', 5);

%% Extended Kalman

input = [-1.5 2.0 1.0];

dynamic_model = [0 1 l; ...
                 -1*cosd(30) -1*sind(30) l; ...
                 cosd(30) -1*sind(30) l];

predicted_points = [];
actual_points = [];
kalman_points = [];
predicted = [0 0 0 0 0 0];
actual = [0 0 0 0 0 0];
kalman = [0 0 0 0 0 0];
dt = 1 / refresh_rate;
for i=1:15 * refresh_rate
    rotational_matrix_predicted = [cos(predicted(3)+pi/2) sin(predicted(3)+pi/2) 0; ...
                                   -1 * sin(predicted(3)+pi/2) cos(predicted(3)+pi/2) 0; ...
                                   0 0 1];
    rotational_matrix_actual = [cos(actual(3)+pi/2) sin(actual(3)+pi/2) 0; ...
                                -1 * sin(actual(3)+pi/2) cos(actual(3)+pi/2) 0; ...
                                0 0 1];
    actual_velocity = r * inv(dynamic_model * rotational_matrix_actual) * input';
    x1 = predicted + [dt *  actual_velocity; 0; 0; 0]';
    latest_actual = actual + [dt * r * inv(dynamic_model * rotational_matrix_predicted) * input'; 0; 0; 0]';
    latest_predicted = x1 + [normrnd(0, 0.01); normrnd(0, 0.01); normrnd(0, 0.1*pi()/180); 0 ; 0; 0]';
    
    theta = kalman(3);
    w1 = -1.5;
    w2 = -2.0;
    w3 = -1;
    Ad = [1 0 0 r*dt * -(2*w1*sin(pi/2 + theta))/(3*(cos(pi/2 + theta)^2 + sin(pi/2 + theta)^2)) r*dt*-(w2*(3*cos(pi/2 + theta) - 3^(1/2)*sin(pi/2 + theta)))/(3*(3^(1/2)*cos(pi/2 + theta)^2 + 3^(1/2)*sin(pi/2 + theta)^2)) r*dt*(w3 *(3*cos(pi/2 + theta) + 3^(1/2)*sin(pi/2 + theta)))/(3*(3^(1/2)*cos(pi/2 + theta)^2 + 3^(1/2)*sin(pi/2 + theta)^2));...
          0 1 0 r*dt * (2*w1*cos(pi/2 + theta))/(3*(cos(pi/2 + theta)^2 + sin(pi/2 + theta)^2)) r*dt*-(w2*(3*sin(pi/2 + theta) + 3^(1/2)*cos(pi/2 + theta)))/(3*(3^(1/2)*cos(pi/2 + theta)^2 + 3^(1/2)*sin(pi/2 + theta)^2)) r*dt*(w3*(3*sin(pi/2 + theta) - 3^(1/2)*cos(pi/2 + theta)))/(3*(3^(1/2)*cos(pi/2 + theta)^2 + 3^(1/2)*sin(pi/2 + theta)^2));...
          0 0 1 r*dt*(10*w1)/9 r*dt*(10*w2)/9 r*dt*(10*w3)/9;
          0 0 0 1 0 0;
          0 0 0 0 1 0;
          0 0 0 0 0 1];
    
    S = eye(6);
    R = [0.01 0 0 0 0 0;...
         0 0.01 0 0 0 0;...
         0 0 0.1*pi()/180 0 0 0;...
         0 0 0 0 0 0;...
         0 0 0 0 0 0;...
         0 0 0 0 0 0];
    Q = [0.5 0 0;% 0 0 0;...
         0 0.5 0;% 0 0 0;...
         0 0 10*pi()/180];% 0 0 0;...
         %0 0 0 0 0 0;...
         %0 0 0 0 0 0;...
         %0 0 0 0 0 0];
    mup = Ad * kalman';
    Sp = Ad * S * Ad' + R;
    
    Ht = [1 0 0 0 0 0;
          0 1 0 0 0 0;
          0 0 1 0 0 0];
          %0 0 0 0 0 0;
          %0 0 0 0 0 0;
          %0 0 0 0 0 0];
    
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    
    kalman = mup + K * ([MeasurementModel(actual(1), actual(2), actual(3))'] - [MeasurementModel(mup(1), mup(2), mup(3))']);
    kalman = kalman';
    S = (eye(6)-K*Ht)*Sp;
    
    predicted = latest_predicted;
    actual = latest_actual;
    predicted_points(:,i) = predicted;
    actual_points(:,i) = actual;
    kalman_points(:,i) = kalman;
end

% Plot
title = 'Motion Model of three-wheeled robot moving in a circle';
PlotPoints(x0, predicted_points, actual_points, kalman_points, title, 'equal', 6);