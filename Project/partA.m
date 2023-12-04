%% part A Handout 2
clear all
clc
close all

qr=0.00313889;
qu=0.00627778;
qt=0.00418519;

A=[-qr-qu,0,0,0;
    qu,-qt,0,0;
    1/4*qr,0,-qr-qu,0;
    0,0,qu,-qt];
B=[1,0;0,0;0,1;0,0];
C=[0.2*qr,0,0.15*qr,0];
D=[0, 0];


V=eig(A);
disp("Eigen Values:")
disp(V)

Asym = sym(A);
J = jordan(Asym);
disp("Jordan Form:")
disp(double(J))

[b1,a1] = ss2tf(A,B,C,D,1);
[b2,a2] = ss2tf(A,B,C,D,2);
r = roots(a1);
disp("Transfer function b1:"),disp(b1)
disp("Transfer function b2:"),disp(b2)
disp("Transfer function a:"),disp(a1)
disp("Transfer function poles:"),disp(r')


disp("u1,   0,      u2,     0")
disp((A*[60000;90000;60000;90000])')

%% impulse
system=ss(A,B,C,D);
t=1:800;
imp=t*0;imp(1)=1;
[y,tOut,x] = impulse(system,t);
figure(1), clf
subplot(4,2,1), hold on
stem(tOut,imp,"k","linewidth",1), title("Impulse to u_1"), legend("Input")
subplot(4,2,3), hold on
plot(tOut,y(:,1),"b","linewidth",1),legend("Output")
subplot(4,2,5), hold on
plot(tOut,x(:,1,1),"c")
plot(tOut,x(:,2,1),"g"),legend("Vs_1","Vc_1")
subplot(4,2,7), hold on
plot(tOut,x(:,3,1),"c")
plot(tOut,x(:,4,1),"g"),legend("Vs_2","Vc_2")
xlabel("Time (hour)")

subplot(4,2,2), hold on
stem(tOut,imp,"k","linewidth",1), title("Impulse to u_2"),legend("Input")
subplot(4,2,4), hold on
plot(tOut,y(:,2),"b","linewidth",1),legend("Output")
subplot(4,2,6), hold on
plot(tOut,x(:,1,2),"c")
plot(tOut,x(:,2,2),"g"),legend("Vs_1","Vc_1")
subplot(4,2,8), hold on
plot(tOut,x(:,3,2),"c")
plot(tOut,x(:,4,2),"g"),legend("Vs_2","Vc_2")
xlabel("Time (hour)")
%% homogenous
xInit=[60000;90000;60000;90000];
[yDry,tOut,xDry] = initial(system,xInit);
figure(2),clf
subplot(3,1,1), hold on, title("Homogenous Response")
plot(tOut,yDry,"b","linewidth",1),legend("Output")
subplot(3,1,2), hold on
plot(tOut,xDry(:,1),"c")
plot(tOut,xDry(:,2),"g"),legend("Vs_1","Vc_1")
subplot(3,1,3), hold on
plot(tOut,xDry(:,3),"c")
plot(tOut,xDry(:,4),"g"),legend("Vs_2","Vc_2")
xlabel("Time (hour)")
%% Handout #3
% 1.1
