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
PlotPoints(x0, predicted_points, actual_points, title, 'equal', 5);