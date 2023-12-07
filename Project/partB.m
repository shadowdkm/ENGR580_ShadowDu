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

C=zeros(3,10);
C(1,1)=1;
C(2,2)=1;
C(3,3)=1;

D=zeros(3);

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
R=0.0001;

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
x0=randn(n,1).*[1,1,1,1,1,1,0.1,0.1,0,0]'*1; %Initial Condition
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