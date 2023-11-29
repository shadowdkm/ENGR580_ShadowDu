clc;clear;close all
%% System's properties
m = 1; % mass
c = 1; % damping coefficient
k = 1; % stiffness
%% State-Space Matrices
n=2; % number of state
m=1; % number of inputs
p=1; % number of outputs
A=zeros(n,n);B=zeros(m,1);C=zeros(p,n); D = zeros(p,m); %#ok
A = [0 1;-k/m -c/m];
B = [1/m]; %#ok
C = [1 0];
D = 0;

T_f=20;
T_s=0.01;


x(:,1) = [1;0];
u = 0;
for i=1:floor(T_f/T_s)
    
    t(i)=(i-1)*T_s;
    %u = sin(t(i));
    x(:,i+1) = x(:,i) + T_s*(A*x(:,i) + B*u);
    y(:,i) = C*x(:,i);
end
hold on
plot(t,y)
    
