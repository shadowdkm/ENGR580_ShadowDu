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
    %u(:,i) = sin(t(i)); % Forced Response
    x(:,i+1) = x(:,i) + T_s*(A*x(:,i) + B*u(:,i));
    y(:,i) = C*x(:,i); %#ok
end
% 
% hold on
% plot(t,y)

%% 
rank(ctrb(A,B))
rank(obsv(A,C))
Q=eye(n);
R=1*3000;

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

figure,
subplot(3,1,1)
plot(t,x(1:3,2:end)')
legend("Position X","Position Y","Position Z")
subplot(3,1,2)
plot(t,x(4:6,2:end)')
legend("Velocity X","Velocity Y","Velocity Z")
subplot(3,1,3)
plot(t,x(7:8,2:end)')
legend("Phi","Theta")
% 
% figure
% drone_Animation(x(1,:),x(2,:),x(3,:),x(7,:),x(8,:),x(1,:)*0);

%% run simulation
% plotSimulation