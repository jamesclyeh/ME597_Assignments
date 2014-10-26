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

x = MoveRobot(x0, l, r, input, refresh_rate, time);

% Plot
title = ('Motion Model of Distribution Free two-wheeled robot spinning in a circle');
PlotPoints(x0, x, title, axis, 1);

%% move in straight line

input = [0 -1 1];

x = MoveRobot(x0, l, r, input, refresh_rate, time);

% Plot
title = 'Motion Model of Distribution Free two-wheeled robot moving in a straight line';
PlotPoints(x0, x, title, axis, 2);

%% move in circle

input = [-1.5 2.0 1.0];

x = MoveRobot(x0, l, r, input, refresh_rate, 15);

% Plot
title = 'Motion Model of Distribution Free two-wheeled robot moving in a circle line';
PlotPoints(x0, x, title, axis, 3);

%% move in circle with 2m diameter

input = ((1 / r) * dynamic_model * [1 0 1]')';

x = MoveRobot(x0, l, r, input, refresh_rate, time);

% Plot
title = 'Motion Model of Distribution Free two-wheeled robot moving in a circle line';
PlotPoints(x0, x, title, 'equal', 4);

%% model with additive gaussian disturbance
