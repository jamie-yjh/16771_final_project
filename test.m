clc
close all;
%% Set up robot parameters
syms Ip Iw IM Mw m M;
syms d r l;
Ip = 0.15;
Iw = 0.5 * 0.4 * 0.1^2;
IM = 0;
Mw = 0.4;
m = 10;
M = 0;
r = 0.1;
l = 0.7;
d = 0.35;
params = struct(...
    'Ip',                Ip, ...
    'Iw',                Iw,...
    'IM',                IM,...
    'Mw',                Mw, ...
    'm',                 m, ...
    'M',                 M,...
    'd',                 d, ...
    'r',                 r, ...
    'l',                 l, ...
    'g',                 9.8);

%% Set the simualation parameters
time_initial = 0; 
time_final = 3;
time_step = 0.01; % sec
time_vec = time_initial:time_step:time_final;
max_iter = length(time_vec);

%% Create the state vector
state = zeros(6,1);
state(1)  = 0;    %theta
state(2)  = 0.3;    %alpha
state(3)  = 0;    %x
state(4)  = 0;    %theta_dot
state(5)  = 0;    %alpha_dot
state(6)  = 0;    %x_dot
% state(7)  = 0;    %theta_ddot
% state(8)  = 0;    %alpha_ddot
% state(9)  = 0;    %x_ddot

%% Loop through the timesteps and update
history = zeros(6,max_iter-1);
tau = zeros(6,max_iter-1);
for iter = 1:max_iter-1
    
    theta = state(1);
    alpha = state(2);
    alpha_dot = state(5);
    x = state(3);
    
    kp = 10;
    kd = 1;
    
    taul = kp*alpha + kd * alpha_dot;
    taur = kp*alpha + kd * alpha_dot;
    tau(:,iter) = taul;
    dr = 0;
    dl = 0;
    timeint = time_vec(iter:iter+1);
    

    [tsave, xsave] = ode45(@(t,s) dynamic(params, state, taul, taur, dl, dr), timeint, state);
    state = xsave(end, :)';
    history(:,iter) = state;
end

%% plot
subplot(3,2, 1);
plot(1:max_iter-1,history(2,:));
title("alpha")
subplot(3,2, 3);
plot(1:max_iter-1,history(5,:));
title("alpha-dot")
subplot(3,2, 2);
plot(1:max_iter-1,history(3,:));
title("x")
subplot(3,2, 4);
plot(1:max_iter-1,history(6,:));
title("x-dot")
figure()
plot(1:max_iter-1,tau);
hold on
plot(1:max_iter-1,history(2,:));
hold on
plot(1:max_iter-1,history(5,:));