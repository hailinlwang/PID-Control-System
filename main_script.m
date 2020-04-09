clear; close all; clc;
% Simulation parameters *** Edit these ***
N=100; % Number of seconds of robot movement to simulate
pps=1000; % Resolution of simulation (points per second)

% Control parameters *** Edit these ***
spd = 0.2; % Initial speed of robot (m/s)
dT = 0.025; % Time interval between controller changes (s)
% Note - make sure your control interval is a multiple of your simulation frequency! Otherwise this won't work.
% E.g. Sim frequency = 1000Hz; Control frequency = 100Hz = check every 10 intervals of simulation.
ctrl_enable=6 ; % Enable control. 0 = no control; 1 = proportional control; 2 = PID control; 3 = PI; 4 = compass
                                 %5 = Compass PID, 6 = D only; 7 compass integration;
                                 %8 = I only
                                 %integration
                                
% Change length of simulation to visualize for D and I only
if ctrl_enable == 5 || ctrl_enable == 6 || ctrl_enable == 8 
   N = 250; 
end

%Toggle Noise filter
f=0;
qf = 0;

% Robot parameters
robot.r = 0.05;    % wheel radius (m)
robot.wMax = 1.16;     % max motor angular velocity (rotations/second)
robot.d = 0.3;      % distance between wheels (m)

robot.iTerm = 0;    % Collection of integral terms
robot.olderror = 0; % Used to calculate derivative term
robot.heading = [1 0]; %Variable to use for compass
robot.yPos = 0; % Deviation from X axis used to reset integral term if robot returns to zero to prevent integral windup

% robot.rlastSpeed = spd; % Starting speed is this
% robot.llastSpeed = spd; % Starting speed is this



% Initalize simulation - No need to change this!
robot.setSpeed = spd; % Set desired speed m/s
T=linspace(0,N,pps*N); % Create vector of time points
robot.path = zeros(2,pps*N); % x,y position of robot at all simulated time points
robot.dir = zeros(2,pps*N); robot.dir(:,1)=[1 0]; % robot starts off pointing in the x direction
robot.lWheel=zeros(2,pps*N); robot.lWheel(:,1) = [0; -robot.d/2]; % track wheel positions L / R
robot.rWheel=zeros(2,pps*N); robot.rWheel(:,1) = [0; +robot.d/2];
robot.encL=zeros(1,pps*N); robot.encR=zeros(1,pps*N); % Simulate encoder readings

robot.enc_error=zeros(1,pps*N);
robot.pwm=zeros(2,pps*N);
robot.yPos_array=zeros(1,pps*N); robot.y_array(:,1) = 0; %Error plot for encoder

% Run simulation *** Implement your control schemes inside this function *** 
robot=drive_robot(robot,T,dT,ctrl_enable,f);

% Plot and analyse results *** Edit this section ***

figure;
hold on;
scatter(robot.path(1,:),robot.path(2,:),'b');
scatter(robot.lWheel(1,:),robot.lWheel(2,:),'k');
scatter(robot.rWheel(1,:),robot.rWheel(2,:),'k');
axis tight
plot(xlim, [0 0], '-r')
ylim([-0.5 0.65])
hold off;
title('Path travelled by robot');
xlabel('x (m)'); ylabel('y (m)');


%Quick filter
if qf == 1;
    windowSize = 2;
    b = (1/windowSize)*ones(1,windowSize);
    a = 1;
    
    y1 = filter(b,a,robot.encL);
    y2 = filter(b,a,robot.encR);
    robot.encL = y1;
    robot.encR = y2;
end
%PLOT ERRORS
robot.enc_error = robot.encL - robot.encR;
figure;
hold on
% scatter(T*spd,robot.pwm(1,:),'k');
% scatter(T*spd,robot.pwm(2,:),'r');
scatter(T*spd,robot.enc_error(1,:),'r');
axis tight
plot(xlim, [0 0], '-r')
ylim([-0.02 0.02])
%plot((T*spd),robot.yPos_array),'g';
%plot((T*spd),robot.enc_error);

hold off
title('Error between left and right encoder values');
xlabel('x (m)'); ylabel('Error (m)');

% % visualize noise 2 - Encoder Error
% x=NaN(1,100);
% for i=1:100
%     x(i)=1+sqrt(0.0001)*randn; 
% end
% figure; plot(x);
% 
% % visualize noise 1 - Positional Disturbance
% x=NaN(1,100);
% for i=1:100
%     x(i)=1+sqrt(0.01)*randn;
% end
% figure; plot(x);
