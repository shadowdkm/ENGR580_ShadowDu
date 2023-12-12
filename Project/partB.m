clc;clear;close all
%% System's properties
mass = 0.1; I = 1e-4; l =0.1; g=9.81;

n=10;

A=zeros(10,10);
A(1,4)=1;
A(2,5)=1;
A(3,6)=1;
A(4,8)=-g;
A(5,7)=g;
A(7,9)=1;
A(8,10)=1;

B=zeros(10,3);
B(6,1)=-1/mass;
B(9,2)=1/I;
B(10,3)=1/I;

C=zeros(7,10);
C(1,9)=1;       % gyroscope
C(2,10)=1;      % gyroscope
C(3,3)=1;       % barometer
C(4,4)=1;       % integrated x acc
C(5,5)=1;       % integrated y acc
% C(?,6)=1;       % integrated z acc, not reliable


D=zeros(size(C,1),size(B,2));

Asym = sym(A);
J = jordan(Asym);
disp("Jordan Form:")
disp(double(J))

%% Simulation Loop

T_f=5;T_s=0.01; % Final Time and sampling rate

x0=randn(n,1); %Initial Condition
x(:,1) = x0;

for i=1:floor(T_f/T_s)
    t(i)=(i-1)*T_s; %#ok %Time
    u(:,i)  = [0;0;0]; %#ok % Free response
    x(:,i+1) = x(:,i) + T_s*(A*x(:,i) + B*u(:,i));
    y(:,i) = C*x(:,i); %#ok
end

%% 
rank(ctrb(A,B))
rank(obsv(A,C))
Q=eye(n);
R=0.001;

[K,S,P] = lqr(A,B,Q,R);
              % x   y   z   vx  vy  vz  phi  theta  phidot thetadot
% K = place(A,B,[-11 -12 -100 -5 -6  -20  -10   -10     -5     -5]);

% x(:,1) = [0.001,0,0,0,0,0,0,0,0,0];
x(:,1) =randn(n,1);

for i=1:floor(T_f/T_s)
    t(i)=(i-1)*T_s; %Time
    u(:,i) = [mass*g;0;0]-K*x(:,i);
    x(:,i+1) = x(:,i) + T_s*(A*x(:,i) + B*u(:,i));
    y(:,i) = C*x(:,i); 
end

figure(1),
subplot(3,1,1)
plot(t,x(1:3,2:end)'), title("Open loop Responce")
legend("Position X","Position Y","Position Z")
subplot(3,1,2)
plot(t,x(4:6,2:end)')
legend("Velocity X","Velocity Y","Velocity Z")
subplot(3,1,3)
plot(t,x(7:8,2:end)')
legend("Phi","Theta"), xlabel("Time (s)")
%% 3.2 Jordan form
J = jordan(A);
eig(A)
% 3.5 BIBO
droneSys=ss(A,B,C,D);
tf(droneSys)
disp("Zero poles, not BIBO stable")
%% 3.7 Homogenous Response
stepVal=[0,0,0];
duration=10;
x0=randn(n,1).*[1,1,0,1,1,1,0.1,0.1,0,0]'*1; %Initial Condition
try
   out = sim('drone_5DOF.slx',duration);
catch exception
   disp(exception)
   return
end
figure(5)
plotDroneSimulationResult(out)
xlabel("Time (t) Homogenous Response")
%% 3.8.1 Step Response: Step in Pulling Force
stepVal=[1,0,0];
duration=10;
x0=zeros(10,1); %Initial Condition
try
   out = sim('drone_5DOF.slx',duration);
catch exception
   disp(exception)
   return
end
figure(6)
plotDroneSimulationResult(out)
xlabel("Time (t) Step in Pulling Force")
%% 3.8.2 Step Response: Step in TauX
stepVal=[0,1,0];
duration=10;
x0=zeros(10,1); %Initial Condition
try
   out = sim('drone_5DOF.slx',duration);
catch exception
   disp(exception)
   return
end
figure(7)
plotDroneSimulationResult(out)
xlabel("Time (t) Step in Tau_X")
%% 3.8.3 Step Response: Step in TauY
stepVal=[0,0,1];
duration=10;
x0=zeros(10,1); %Initial Condition
try
   out = sim('drone_5DOF.slx',duration);
catch exception
   disp(exception)
   return
end
figure(8)
plotDroneSimulationResult(out)
xlabel("Time (t) Step in Tau_Y")
%% 2.1 Controlable
% 2.3 
Bbroken=B; Bbroken(10,3)=0;
rank(ctrb(A,Bbroken))
% [T,Abar,Bbar,Cbar] = Kalman_Decomposition(A,Bbroken,C);
[Abar,Bbar,Cbar,T,k] = ctrbf(A,Bbroken,C);
A_uncontrolable=Abar(1:4,1:4);
eig(A_uncontrolable) %=> all zero, stablizable

%% Observability
% 2.1 Observable? no
rank(obsv(A,C))
% 2.2 Kalman decomposation
[T,Abar,Bbar,Cbar]=Kalman_Decomposition(A,B,C);
% Add GPS to get XYZ position
Cob=C;
% C(1,9)=1;       % gyroscope
% C(2,10)=1;      % gyroscope
% C(3,3)=1;       % barometer
% C(4,4)=1;       % integrated x acc
% C(5,5)=1;       % integrated y acc
Cob(6,1)=1;       % GPS x postion
Cob(7,2)=1;       % GPS y postion
rank(obsv(A,Cob))
Ki=zeros(3,7);    % no tracking gain
Kp=zeros(3,7);    % no tracking gain
ObservedVariance=zeros(7,1); % no noise in measurments
DisturbanceVariance=zeros(10,1);% no disturbance
TrackingEnabled=0;% no tracking

Q_duo=eye(n);
R_duo=0.0001;
[K_duo,S_duo,P_duo] = lqr(A',Cob',Q_duo,R_duo);
L=K_duo';
x0=randn(n,1).*[1,1,0,1,1,1,0.1,0.1,0,0]'*1; %Initial Condition
duration=10;
% Linear Observer State Feedback Control
try
   out = sim('drone_5DOF_observer.slx',duration);
catch exception
   disp(exception)
   return
end

figure(9),clf
plotDroneObserverResult(out)
xlabel("Time (t) Output Feedback")
%% Tracking 
TrackingEnabled=1;
duration=20;

beta=-100;
Ki=zeros(3,7);
Ki(1,3)=beta;

Kp=zeros(3,7);
Kp(1,3)=-3000;

try
   out = sim('drone_5DOF_observer.slx',duration);
catch exception
   disp(exception)
   return
end
figure(10),clf
plotDroneObserverResult(out)
%% Tracking with Disturbance of wind on x axis
beta=-100;
Ki=zeros(3,7);
Ki(1,3)=beta;

Kp=zeros(3,7);
Kp(1,3)=-3000;

ObservedVariance=zeros(7,1); 
DisturbanceVariance=zeros(10,1);
DisturbanceVariance(4)=1; % x velocity
% DisturbanceVariance(5)=1; % y velocity
% DisturbanceVariance(6)=1; % z velocity
% DisturbanceVariance(9)=1; % x rotation velocity
% DisturbanceVariance(10)=1;% y rotation velocity
try
   out = sim('drone_5DOF_observer.slx',duration);
catch exception
   disp(exception)
   return
end
figure(11),clf
plotDroneObserverResult(out)
%% Tracking with Disturbance of wind on x rotation
beta=-100;
Ki=zeros(3,7);
Ki(1,3)=beta;

Kp=zeros(3,7);
Kp(1,3)=-3000;

ObservedVariance=zeros(7,1); 
DisturbanceVariance=zeros(10,1);
%DisturbanceVariance(4)=1; % x velocity
% DisturbanceVariance(5)=1; % y velocity
% DisturbanceVariance(6)=1; % z velocity
 DisturbanceVariance(9)=1; % x rotation velocity
% DisturbanceVariance(10)=1;% y rotation velocity
try
   out = sim('drone_5DOF_observer.slx',duration);
catch exception
   disp(exception)
   return
end
figure(12),clf
plotDroneObserverResult(out)
%% Tracking a route with Disturbance of wind on z axis
beta=-100;
Ki=zeros(3,7);
Ki(1,3)=beta;

Kp=zeros(3,7);
Kp(1,3)=-3000;

ObservedVariance=zeros(7,1); 
DisturbanceVariance=zeros(10,1);
%DisturbanceVariance(4)=1; % x velocity
% DisturbanceVariance(5)=1; % y velocity
 DisturbanceVariance(6)=1; % z velocity
% DisturbanceVariance(9)=1; % x rotation velocity
% DisturbanceVariance(10)=1;% y rotation velocity
try
   out = sim('drone_5DOF_observer.slx',duration);
catch exception
   disp(exception)
   return
end
figure(13),clf
plotDroneObserverResult(out)
%% Regularization with Measurement Noise
TrackingEnabled=1;

beta=-100;
Ki=zeros(3,7);
Ki(1,3)=beta;

Kp=zeros(3,7);
Kp(1,3)=-3000;
DisturbanceVariance=zeros(10,1);
ObservedVariance(1)=0.005/180*pi/10; % Gyroscope of MPU6050, from spec
ObservedVariance(2)=0.005/180*pi/10; % Gyroscope of MPU6050, from spec
ObservedVariance(3)=1.1/sqrt(2);     % Barometer SPL-001, from spec
ObservedVariance(4)=0.04*9.8/10;     % Accelerometer of MPU6050, from spec
ObservedVariance(5)=0.04*9.8/10;     % Accelerometer of MPU6050, from spec
ObservedVariance(6)=3.9;             % GPS x, hand measured
ObservedVariance(7)=3.9;             % GPS y, hand measured
try
   out = sim('drone_5DOF_observer.slx',duration);
catch exception
   disp(exception)
   return
end
figure(14),clf
plotDroneObserverResult(out)
%% Tracking a route with Measurement Noise
TrackingEnabled=1;
try
   out = sim('drone_5DOF_observer.slx',30);
catch exception
   disp(exception)
   return
end
figure(15),clf,plotDroneObserverResult(out)
figure(16),clf
plot3(-out.simuTargetX,-out.simuTargetY,out.simuTargetZ,'k'), hold on
simuReal2bEstOutput=reshape(out.simuReal2bEstOutput,7,[])';
plot3(simuReal2bEstOutput(:,6),simuReal2bEstOutput(:,7),simuReal2bEstOutput(:,3)), hold off
legend("Target Route","Tracking Result")
xlabel("x (m)")
ylabel("y (m)")
zlabel("z (m)")
axis equal
grid minor

