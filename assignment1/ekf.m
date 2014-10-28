% Extended Kalman filter example
clear;clc;

%% Create AVI object
makemovie = 1;
if(makemovie)
    vidObj = VideoWriter('ekf.avi');
    vidObj.Quality = 100;
    vidObj.FrameRate = 8;
    open(vidObj);
end

% Discrete time step
dt = 0.1;

% Initial State
x0 = [0 0 0 -1.5 2.0 1.0]';
w1 = -1.5;
w2 = 2.0;
w3 = 1;

% Prior
mu = [-2 3 -2 -5 5 5]'; % mean (mu)
S = eye(6);% covariance (Sigma)

% Discrete motion model


R = [0.01 0 0 0 0 0;...
     0 0.01 0 0 0 0;...
     0 0 0.1*pi()/180 0 0 0;...
     0 0 0 0 0 0;...
     0 0 0 0 0 0;...
     0 0 0 0 0 0];
[RE, Re] = eig (R);

% Measurement model defined below
Q = [0.5 0 0;% 0 0 0;...
     0 0.5 0;% 0 0 0;...
     0 0 10*pi()/180];

% Simulation Initializations
Tf = 50;
T = 0:dt:Tf;
n = 6;
x = zeros(n,length(T));
x(:,1) = x0;
m = length(Q(:,1));
y = zeros(m,length(T));
mup_S = zeros(n,length(T));
mu_S = zeros(n,length(T));
r = 0.25;

%% Main loop
for t=2:length(T)
    %% Simulation
    
    %{
    if mod(t, 10) == 1
        Q = [0.01 0 0;% 0 0 0;...
             0 0.01 0;% 0 0 0;...
             0 0 10*pi()/180];
    else
        Q = [0.5 0 0;% 0 0 0;...
             0 0.5 0;% 0 0 0;...
             0 0 10*pi()/180];
    end
    %}
    
    % Update state
    theta = x(3,t-1);
    Ad = [1 0 0 r*dt * -(2*sin(pi/2 + theta))/(3*(cos(pi/2 + theta)^2 + sin(pi/2 + theta)^2)) r*dt*-((3*cos(pi/2 + theta) - 3^(1/2)*sin(pi/2 + theta)))/(3*(3^(1/2)*cos(pi/2 + theta)^2 + 3^(1/2)*sin(pi/2 + theta)^2)) r*dt*((3*cos(pi/2 + theta) + 3^(1/2)*sin(pi/2 + theta)))/(3*(3^(1/2)*cos(pi/2 + theta)^2 + 3^(1/2)*sin(pi/2 + theta)^2));...
          0 1 0 r*dt * (2*cos(pi/2 + theta))/(3*(cos(pi/2 + theta)^2 + sin(pi/2 + theta)^2)) r*dt*-((3*sin(pi/2 + theta) + 3^(1/2)*cos(pi/2 + theta)))/(3*(3^(1/2)*cos(pi/2 + theta)^2 + 3^(1/2)*sin(pi/2 + theta)^2)) r*dt*((3*sin(pi/2 + theta) - 3^(1/2)*cos(pi/2 + theta)))/(3*(3^(1/2)*cos(pi/2 + theta)^2 + 3^(1/2)*sin(pi/2 + theta)^2));...
          0 0 1 r*dt*(10)/9 r*dt*(10)/9 r*dt*(10)/9;
          0 0 0 1 0 0;
          0 0 0 0 1 0;
          0 0 0 0 0 1];
    
    %{
    jacobian = [1 0 0 dt 0 0;...
                0 1 0 0 dt 0;...
                0 0 1 0 0 dt;...
                0 0 (w1*sin(theta))/6 - (w2*sin(theta))/12 - (w3*sin(theta))/12 + (3^(1/2)*cos(theta)*w2)/12 - (3^(1/2)*cos(theta)*w3)/12 1 0 0;...
                0 0 (cos(theta)*w2)/12 - (cos(theta)*w1)/6 + (cos(theta)*conj(w3))/12 + (3^(1/2)*w2*sin(theta))/12 - (3^(1/2)*w3*sin(theta))/12 0 1 0;...
                0 0 1 0 0 1];
    %}
                
    x(:,t) = Ad*x(:,t-1) + [normrnd(0, 0.01); normrnd(0, 0.01); normrnd(0, 0.1*pi()/180); 0 ; 0; 0];

    % Take measurement
    % Select a motion disturbance
    % Determine measurement
    y(:,t) = MeasurementModel(x(1,t), x(2,t), x(3,t), t);


    %% Extended Kalman Filter Estimation
    % Prediction update
    mup = Ad*mu;
    %Sp = jacobian*S*jacobian' + R;
    Sp = Ad*S*Ad' + R;

    % Linearization
    Ht = [1 0 0 0 0 0;
          0 1 0 0 0 0;
          0 0 1 0 0 0];

    % Measurement update
    K = Sp*Ht'*inv(Ht*Sp*Ht'+Q);
    mu = mup + K*(y(:,t)-[mup(1); mup(2); mup(3)]);
    S = (eye(6)-K*Ht)*Sp;

    % Store results
    %mup_S(:,t) = mup;
    mu_S(:,t) = mu;
    %K_S(:,t) = K;


    %% Plot results
    figure(1);clf; hold on;
    plot(0,0,'bx', 'MarkerSize', 6, 'LineWidth', 2)
    plot(x(1,2:t),x(2,2:t), 'ro--')
    plot(mu(1), mu(2), 'ko--')
    plot(mu_S(1,2:t),mu_S(2,2:t), 'bx--')
    mu_pos = [mu(1) mu(2)];
    S_pos = [S(1,1) S(1,2); S(2,1) S(2,2)];
    error_ellipse(S_pos,mu_pos,0.75);
    error_ellipse(S_pos,mu_pos,0.95);
    title('True state and belief')
    axis equal
    axis([-2 3 -2 3])
    %if (makemovie) writeVideo(vidObj, getframe(gca)); end

end
if (makemovie) close(vidObj); end

figure(2)
hold all;
title('X position')
plot(T, x(1,:), '-b');
plot(T, mu_S(1,:), '-r');
figure(3)
hold all;
title('Y position')
plot(T, x(2,:), '-b');
plot(T, mu_S(2,:), '-r');
figure(4)
hold all;
title('Theta position')
plot(T, x(3,:), '-b');
plot(T, mu_S(3,:), '-r');
figure(5)
hold all;
title('w1 velocity')
plot(T, x(4,:), '-b');
plot(T, mu_S(4,:), '-r');
figure(6)
hold all;
title('w2 velocity')
plot(T, x(5,:), '-b');
plot(T, mu_S(5,:), '-r');
figure(7)
hold all;
title('w3 velocity')
plot(T, x(6,:), '-b');
plot(T, mu_S(6,:), '-r');
