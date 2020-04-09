% Edit this function to implement alternative control schemes
%  *** Add code in the switch statement starting at line 19 ***
function robot=drive_robot(robot,T,dT,ctrl_enable,f)

% Initializing
DC_ideal=getDC(robot.setSpeed,robot.r,robot.wMax); % Get duty cycle to initialize motors
robot.rMot = DC_ideal;
robot.lMot = DC_ideal;

% Working out timings
Tstep=T(2); % Step size of simulation
ctrl_interval=round(dT/Tstep);

% Control parameters 
kP = 0.0; 
kI = 0.04;
kD = 0.008;
kC = 0.01;

%Parameters for control 1 PID
kP1 = 25; 
%Parameters for control 2 PID
kP2 = 0.2; 
kI2 = 0.02;
kD2 = 0.008; 

%Parameters for control 4 P, compass
kP4 = 0.02;

%Parameters for control 5 PID, compass
kP5 = 0.02; 
kI5 = 0.02;
kD5 = 0.008; 

%Parameters for control 6 D only
kD6 = 0.065; 

%Parameters for control 7 PI, compass
kP7 = 0.02; 
kI7 = 0.02;

%Parameters for control 8 I only
kI8 = 0.001; 


for i=2:length(T)
    
    % Compute change in position
    robot=newPos(robot,Tstep,i,f);
    robot.yPos = robot.path(2,i); %Deviation from trajectory, used as a GPS
    robot.yPos_array(i) = robot.yPos; 
    
    if mod(i,ctrl_interval)< eps %If it is time to provide feedback
       
%        robot.enc_error(2,i/25) = getError(robot,i,ctrl_interval);
%        robot.enc_error(1,i/25) = i;
       
       switch ctrl_enable % Open loop control; no feedback
           case 0; % Open loop, do nothing
           case 1; % Proportional control
               
               % Get error signal (Computed as Left - Right encoder reading)
               enc_error = getError(robot,i,ctrl_interval);
               % Modify motor duty cycle accordingly
               robot.rMot = robot.rMot+kP1*enc_error;
               robot.lMot = robot.lMot-kP1*enc_error;
               
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);
               
           case 2; % *** PID ***
               
               % Get error signal (Computed as Left - Right encoder reading)
               enc_error = getError(robot,i,ctrl_interval);
               
               % Modify motor duty cycle accordingly
               robot.iTerm = robot.iTerm +(kI2*enc_error);
               dTerm = (enc_error - robot.olderror)/dT;
                                      
               robot.rMot = robot.rMot + (kP2 * enc_error) + robot.iTerm + (kD2 * dTerm);
               robot.lMot = robot.lMot - (kP2 * enc_error) - robot.iTerm - (kD2 * dTerm);
                             
               robot.olderror = enc_error; 
               
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);

            case 3 % *** PI *** 
               % Get error signal (Computed as Left - Right encoder reading)
               enc_error = getError(robot,i,ctrl_interval);
               
               % Modify motor duty cycle accordingly
               robot.iTerm = robot.iTerm +(kI*enc_error);
               dTerm = (enc_error - robot.olderror)/dT;
                          
               robot.rMot = robot.rMot + kP * enc_error + robot.iTerm;
               robot.lMot = robot.lMot - kP * enc_error - robot.iTerm;
               
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);
               
            case 4 % *** Compass *** 
               robot.rMot = robot.rMot + kP4 * robot.heading;
               robot.lMot = robot.lMot - kP4 * robot.heading;
               
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);
               
           case 5 
                % Get error signal (Computed as Left - Right encoder reading)
               enc_error = getError(robot,i,ctrl_interval);
               
               % Modify motor duty cycle accordingly
               robot.iTerm = robot.iTerm +(kI5*enc_error);
               dTerm = (enc_error - robot.olderror)/dT;
               
               robot.rMot = robot.rMot + kP5 * robot.heading + robot.iTerm + (kD5 * dTerm);
               robot.lMot = robot.lMot - kP5 * robot.heading - robot.iTerm - (kD5 * dTerm);
               
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);
               
           case 6
                % Get error signal (Computed as Left - Right encoder reading)
               enc_error = getError(robot,i,ctrl_interval);
               
               % Modify motor duty cycle accordingly
               dTerm = (enc_error - robot.olderror)/dT;
               
               robot.rMot = robot.rMot + (kD6 * dTerm);
               robot.lMot = robot.lMot - (kD6 * dTerm);
               
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);
           case 7
               % Get error signal (Computed as Left - Right encoder reading)
               enc_error = getError(robot,i,ctrl_interval);
               robot.iTerm = robot.iTerm +(kI7*enc_error);
              
               
               robot.rMot = robot.rMot + kP7 * robot.heading + robot.iTerm;
               robot.lMot = robot.lMot - kP7 * robot.heading - robot.iTerm;
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);
               
           case 8
               % Get error signal (Computed as Left - Right encoder reading)
               enc_error = getError(robot,i,ctrl_interval);
               robot.iTerm = robot.iTerm +(kI8*enc_error);
               
               %To prevent integral windup, lower integral if we return within certain range
               if robot.yPos == 0;
                   robot.iTerm = robot.iTerm*1/2;
               end
               robot.rMot = robot.rMot + robot.iTerm;
               robot.lMot = robot.lMot - robot.iTerm;
               % Make sure PWM is still between [0, 1]
               robot=checkDC(robot);
       end
       
           % Code to visualize robot as it steps
%     clf; hold on;
%     scatter(robot.path(1,i),robot.path(2,i),'b');
%     scatter(robot.lWheel(1,i),robot.lWheel(2,i),'k');
%     scatter(robot.rWheel(1,i),robot.rWheel(2,i),'k');
%     axis tight
%     plot(xlim, [0 0], '-r')
%     xlim([-0.1 1]); ylim([-.2 .2]);
       
    end

end
end
