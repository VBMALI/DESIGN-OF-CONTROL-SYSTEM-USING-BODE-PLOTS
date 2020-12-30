%% Cleanup
clearvars  % Clear the workspace
close all  % Close all open figures
clc   % Clear the command window

%% Simulation Parameters

% Solver
simPrm.solTyp = 'Fixed-step';  % Solver type
simPrm.sol    = 'ode3';  % Solver type 2
simPrm.dt  = 0.01; % Integration step size
simPrm.tEnd = 10; % Simulation end time

% Input
Input.Ts = 1;  % Start time of input step
Input.A = 1;   % Amplitude of input step

% Sensor
Tau = 0.005;  % Sensor Time delay
H = tf(1,[Tau 1]);  % Sensor Transfer function

Tau2 = 8;  % Sensor time constant for instability
H2 = tf(1,[Tau2 1]);  % Instability Sensor transfer function

%% Design Requirements
PM = 40;  % Lower limit of phase margin
Threshold = 5;  % Settling Time percentage threshold

%% Anonymous functions for Bode plot characteristics
pAng    = @(X) (1-sind(X))/(1+sind(X));
magDrop = @(X) 10*log10(X);
evalM   = @(g,X) 20*log10(abs(evalfr(g,X*1j) ) );
evalA   = @(g,X) 180/pi*angle( evalfr(g,X*1j) );

%% Error Constant and gain
Kc = 6;  % Set Gain as 6 more than 5
Kv = Kc*200/(25*6); % Calculate Velocity error constant

%% Original Loop Transfer function

g = tf(Kc*50*[1 4],[1 10 49 150 0]);% Create loop transfer function

% figure(1);margin(g);grid;  % Create bode plot of original loop transfer function

pm0 = -30.2; wm0 = 7.04;  % Current phase margin and gain cross over frequency

pmC = (PM - pm0);  % Required change in phase margin

%% Lead & Lag Compensator Design

% Lead Compensator 1 Design
pmC1 = 45;  % Target phase margin for first compensator design
alpha1 = (1-sind(pmC1))/(1+sind(pmC1));  % Calculate alpha
MGDrop1 = magDrop(alpha1);  % Calculate Magnitude drop

wm1 = 9.3;  % Frequency at Magnitude drop

T1 = 1/wm1/sqrt(alpha1);  

D1 = tf([T1 1],[alpha1*T1 1]); % Transfer function of first compensator

%-------------------------------------------------------------------------%
% Lead Compensator 2 Design
pmC2 = pmC-pmC1;   % Remaining phase margin for second lead compensator

alpha2 = (1-sind(pmC2))/(1+sind(pmC2));  % calculate alpha
MGDrop2 = magDrop(alpha2);  % Calculate Magnitude drop

wm2 = 11.1;  % Frequency at this magnitude drop

T2 = 1/wm2/sqrt(alpha2);  

D2 = tf([T2 1],[alpha1*T2 1]); % Transfer function of second compensator

%-------------------------------------------------------------------------%
% Lag Compensator Design

w1 = 7.6; %7.6; 7.69 Frequency at required phase margin

T = 13/w1; %13/w1; 

M = 7.51; %7.51; 7.29

Beta = 10^(M/20);

D3 = tf([T 1],[Beta*T 1]); %  transfer function of third compensator

D = D1*D2*D3;   % Transfer function of entire compensator design

gcl = feedback(g*D,1);  % Transfer function of closed loop system with unity feedback

%% Open and configure the simulink model

open_system('p3Sim.slx');  % Open the simulink model
set_param('p3Sim','SolverType',simPrm.solTyp);  % set this solver type in simulink parameters 
set_param('p3Sim','Solver',simPrm.sol);     % set this integration method in simulink solver

%% Simulate the model with unity feedback

set_param('p3Sim/SW', 'sw', '1');  % Connect unity feedback line to switch input
SimOut1 = sim('p3Sim','SignalLoggingName','sdata');  % Simulate the model and save results in sdata

%% Extract the results for plotting and calculation
%Handy constants
Out = 2;
U = 1;
In = 3;

Results.Y = SimOut1.sdata{Out}.Values.Data(:,1);
Results.U = SimOut1.sdata{U}.Values.Data(:,1);
Results.In = SimOut1.sdata{In}.Values.Data(:,1);

%% Response Characteristics
TSe = stepinfo(gcl,'SettlingTimeThreshold',Threshold/100).SettlingTime;  % Settling Time
[Gm,Pm] = margin(g*D);  % Phase Margin
Umax = max(abs(Results.U)); % Maximum Actuator Force

%% Conclude the controller design
mycontroller(TSe,Pm, Umax,Tau,Tau2);  % Conclude my controller performance

%% Time delay in Sensor on position feedback

% Before performance go out of spec
gcl2 = feedback(g*D,H); % Transfer function of closed loop system with Sensor

%% Simulate the model with Sensor feedback

set_param('p3Sim/SW', 'sw', '0');  % Connect unity feedback line to switch input
SimOut2 = sim('p3Sim','SignalLoggingName','sdata');  % Simulate the model and save results in sdata

Results.U2 = SimOut2.sdata{U}.Values.Data(:,1);  % Actuator force
Results.Y2 = SimOut2.sdata{Out}.Values.Data(:,1);  % Unstable output

TSe2 = stepinfo(gcl2,'SettlingTimeThreshold',Threshold/100).SettlingTime;  % Settling Time
Umax2 = max(abs(Results.U2)); % Maximum Actuator Force

%% Plot figures
% figure(1)
open('documents/figures/KcGL.fig');

%figure(2)
open('documents/figures/KcGLD.fig');

% figure(3);
% myplot(SimOut1.tout, Results.Y,'Step response of Closed loop system with Unity Feedback',1,'b','Time(sec)','Position (m)')
% hold on
% myplot(SimOut1.tout, Results.In,'Step response of Closed loop system with Unity Feedback',1,'k','Time(sec)','Position (m)')
% legend('Output Response','Input Step');yticks(0:0.1:1.2);
% hold off
open('documents/figures/step.fig');

% figure(4);
% rlocus(g*D);axis equal;xlim([-45 10]);ylim([-30 30]);  % Root Locus of closed loop system
% title('Root Locus plot of Lead-Lag Compensator loop transfer function');
open('documents/figures/rlocus.fig');

% figure(5);
% % nyquist(g*D);grid;axis equal;xlim([-3 3]);ylim([-3 3]);  % Nyquist Plot
% % title('Nyquist plot of Lead-Lag Compensator loop transfer function');
open('documents/figures/Nyq.fig');
% 
% figure(6)
% % myplot(SimOut1.tout, Results.U,'Actuator Response for Lead-Lag controller design',1,'r','Time(sec)','Force (N)')
open('documents/figures/Force.fig');

% margin(g*D*H);grid;
% title('Bode Plot of system at time delay causing out of spec performance'); % Bode plot for verifying out of spec
open('documents/figures/BPOutSpec.fig');

% figure(8);
open('documents/figures/Unstable.fig');hold off  % Bode plot of G*D1*D2*D3

% figure(9);
% margin(g*D*H2);grid;
% title('Bode Plot of system at time delay causing instability');  % Bode plot for verifying Instability
open('documents/figures/BPUnstable.fig');

%% Anonymous functions
% function for plotting the figures
function myplot(x,y,ttl,LW,color,xlbl,ylbl)
plot(x,y,color,'LineWidth',LW);
xticks(0:1:10);  % Give the x axis ticks
title(ttl); % Give the title.
xlabel(xlbl);
ylabel(ylbl);
end

function mycontroller(Tse,PMargin, Umax,Tau,Tau2)
fprintf('<strong>Below are the performance characteristics of closed loop system</strong>\n');
if Tse < 1 && PMargin > 40 && Umax < 15
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf('The phase margin is greater than 40 deg. Requirement is Satisfied.\n');
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf('The settling time is less than 1 second. Requirement is Satisfied.\n');
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf('The actuator force is less than 15N. Requirement is Satisfied.\n');
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf('<strong>Controller Design is meeting all the requirements.</strong>\n');
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf('<strong>The maximum delay in position sensor before performance goes out of spec is %f milliseconds.</strong>\n',(Tau*1000)-1);
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf('<strong>The maximum delay in position sensor before system becomes unstable is %f seconds.</strong>\n',Tau2-1);
else
    fprintf('<strong>------------------------------------------------------------------------------</strong>\n');
    fprintf(2,'<strong>One of the requirements is not satisfied.Please redesign the controller.</strong>\n');    
end
end

