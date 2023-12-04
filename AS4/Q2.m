clear all
clc

syms A B w C t
A=[0 1 0 0;3*w*w 0 0 2*w;0,0,0,1;0,-2*w 0 1];
A1=[0 1 0 0;3 0 0 2;0,0,0,1;0,-2 0 1];
B=[0,0;1,0;0,0;0,1];

C=[B A*B A*A*B A*A*A*B];

B1=[0,0;0,0;0,0;0,1];%radial down
C1=[B1 A*B1 A*A*B1 A*A*A*B1];

B2=[0,0;1,0;0,0;0,0];%tangential  down
C2=[B2 A*B2 A*A*B2 A*A*A*B2];

%% c
clear all
w=1;
A=[0 1 0 0;3 0 0 2;0,0,0,1;0,-2 0 1];
B=[0,0;1,0;0,0;0,1];
Co = ctrb(A,B)
rank(Co)

%% e
K = place(A,B,[-5 -6 -7 -8])
C=eye(4);
D=0;
satellite = ss(A-B*K,B,C,D);
xInit=[1;0;2;0];
[y0,tOut,x0] = initial(satellite,xInit);
u=-K*x0';

figure,clf
subplot(3,1,1)
hold on, grid on
plot(tOut,x0(:,1:2))
legend("x1","x2")

subplot(3,1,2)
hold on, , grid on
plot(tOut,x0(:,3:4))
legend("x3","x4")

subplot(3,1,3)
hold on, , grid on
plot(tOut,u(1,:))
plot(tOut,u(2,:))
legend("u1","u2")


