clc;clear; %close all
%% System's properties
mass = 1; I = 1e-4; l =0.1; g=9.81;
%% System properties
n=6;m=2;
% A,B,C matrices
A = zeros(n,n);
B=zeros(n,m);
C=[1 0 0 0 0 0;0 0 1 0 0 0;0 0 0 0 1 0];

A(1,2) = 1;
A(2,5) = -g;
A(3,4) = 1;
A(4,5) = 0;
A(5,6) = 1;

B(4,1) = 1/(mass);
B(6,2) = l/(2*I);

%% Simulation Loop

T_f=20;T_s=0.01; % Final Time and sampling rate

x(:,1) = randn(n,1); %Initial Condition

for i=1:floor(T_f/T_s)
    t(i)=(i-1)*T_s; %#ok %Time
    u(:,i)  = [0;0]; %#ok % Free response
    %u(:,i) = sin(t(i)); % Forced Response
    x(:,i+1) = x(:,i) + T_s*(A*x(:,i) + B*u(:,i));
    y(:,i) = C*x(:,i); %#ok
end

hold on
plot(t,y)

%% Design a Full-State feedback controller

CTRB = rank(ctrb(A,B)) == 6;
OBSV = rank(obsv(A,C)) == 6;

%K = place(A,B,0.1*[-10 -2 -10 -3 -100 -1])
K = place(A,B,[-10 -2 -10 -3 -100 -1])


x(:,1) = [1 0 1 -1 1 1];

for i=1:floor(T_f/T_s)
    t(i)=(i-1)*T_s; %#ok %Time
    u(:,i) = -K*x(:,i);
    x(:,i+1) = x(:,i) + T_s*(A*x(:,i) + B*u(:,i));
    y(:,i) = C*x(:,i); %#ok
end

figure
hold on
plot(t,y)

%% Simulink
out = sim('Simulation.slx',5)
X = squeeze(out.X)
System.t = out.t(1:1:end)
System.state = X([1,3,5],1:1:end)
My_Animation