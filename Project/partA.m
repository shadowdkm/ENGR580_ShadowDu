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
% 1.1 Controlable?
rank([B A*B A*A*B A*A*A*B])
disp("controllable")

% 1.2 Feedback Control
tau=log(0.05)/20;  %exp(-48*tau)=0.05 , but this one is a little bit slow
K = place(A,B,[tau tau*2 tau tau*2]);
feedbackSystem=ss(A-B*K,B,C,D);
t=1:100;
fbxInit=xDry(30000,:);% when the land is dryed from full state for 300 hours
fbxEqu=[60000;90000;60000;90000];
fbu=K*fbxEqu*ones(1,100);
[yFB,tFB,xFB] = lsim(feedbackSystem,fbu,t,fbxInit);
% [yFB,tFB,xFB] = initial(feedbackSystem,xInit);
figure(3), clf
subplot(3,2,1), hold on, grid minor
plot(tFB,xFB(:,1),"c")
plot(tFB,tFB*0+60000*0.95,'k')
plot(tFB,tFB*0+60000*1.05,'k'),legend("Vs_1","95%","105%"), xlabel("Time (h)")
subplot(3,2,2), hold on, grid minor
plot(tFB,xFB(:,2),"g")
plot(tFB,tFB*0+90000*0.95,'k')
plot(tFB,tFB*0+90000*1.05,'k'),legend("Vc_1","95%","105%"), xlabel("Time (h)")
subplot(3,2,3), hold on, grid minor
plot(tFB,xFB(:,3),"c")
plot(tFB,tFB*0+60000*0.95,'k')
plot(tFB,tFB*0+60000*1.05,'k'),legend("Vs_2","95%","105%"), xlabel("Time (h)")
subplot(3,2,4), hold on, grid minor
plot(tFB,xFB(:,4),"g")
plot(tFB,tFB*0+90000*0.95,'k')
plot(tFB,tFB*0+90000*1.05,'k'),legend("Vc_2","95%","105%"), xlabel("Time (h)")
subplot(3,2,5:6), hold on, grid minor
uTau20=-K*(xFB'-fbxEqu*ones(1,100));
plot(tFB,uTau20(1,:)),
plot(tFB,uTau20(2,:)),legend("Sprinkler 1","Sprinkler 2"), xlabel("Time (h)")

% LQR
Q=eye(4);
R=1;
[K_lqr,S,P] = lqr(A,B,Q,R);
lqrSystem=ss(A-B*K_lqr,B,C,D);
LQRu=K_lqr*fbxEqu*ones(1,100);
[yLQR,tLQR,xLQR] = lsim(lqrSystem,LQRu,t,fbxInit);
% [yLQR,tLQR,xLQR] = initial(feedbackSystem,xInit);
figure(4), clf
subplot(3,2,1), hold on, grid minor

plot(tFB,xFB(:,1),"c"),plot(tLQR,xLQR(:,1),"r")

legend("Vs_1 Tau=20","Vs_1 LQR"), xlabel("Time (h)")
subplot(3,2,2), hold on, grid minor

plot(tFB,xFB(:,2),"g"),plot(tLQR,xLQR(:,2),"r")

legend("Vc_1 Tau=20","Vc_1 LQR"), xlabel("Time (h)")
subplot(3,2,3), hold on, grid minor

plot(tFB,xFB(:,3),"c"),plot(tLQR,xLQR(:,3),"r")
legend("Vs_2 Tau=20","Vs_2 LQR"), xlabel("Time (h)")
subplot(3,2,4), hold on, grid minor

plot(tFB,xFB(:,4),"g"),plot(tLQR,xLQR(:,4),"r")
legend("Vc_2 Tau=20","Vc_2 LQR"), xlabel("Time (h)")

uLQR=-K*(xLQR'-fbxEqu*ones(1,100));

subplot(3,2,5), hold on, grid minor

plot(tLQR,uTau20(1,:)),plot(tLQR,uLQR(1,:))
legend("Sprinkler 1 Tau=20","Sprinkler 1 LQR"), xlabel("Time (h)")
subplot(3,2,6), hold on, grid minor

plot(tLQR,uTau20(2,:)),plot(tLQR,uLQR(2,:))
legend("Sprinkler 2 Tau=20","Sprinkler 2 LQR"), xlabel("Time (h)")

%% 1.4 Broken Sprinkler 2
Bbroken=[1,0;0,0;0,0;0,0];
rank([Bbroken A*Bbroken A*A*Bbroken A*A*A*Bbroken])
disp("uncontrollable")
%but according to A and Bbroken, the system should be controllable.

[Abar,Bbar,Cbar,T,k] = ctrbf(A,Bbroken,C);
%Abar(1,1)<0 -> stablizable

% 1.5 Simulation on springkle 2 broken
feedbackSystem=ss(A-Bbroken*K,Bbroken,C,D);
t=1:100;
fbxInit=xDry(30000,:);% when the land is dryed from full state for 300 hours
fbxEqu=[60000;90000;60000;90000];
fbu=K*fbxEqu*ones(1,100);
[yFB,tFB,xFB] = lsim(feedbackSystem,fbu,t,fbxInit);
figure(5), clf
subplot(3,2,1), hold on, grid minor
plot(tFB,xFB(:,1),"c")
plot(tFB,tFB*0+60000*0.95,'k')
plot(tFB,tFB*0+60000*1.05,'k'),legend("Vs_1","95%","105%"), xlabel("Time (h)")
subplot(3,2,2), hold on, grid minor
plot(tFB,xFB(:,2),"g")
plot(tFB,tFB*0+90000*0.95,'k')
plot(tFB,tFB*0+90000*1.05,'k'),legend("Vc_1","95%","105%"), xlabel("Time (h)")
subplot(3,2,3), hold on, grid minor
plot(tFB,xFB(:,3),"c")
plot(tFB,tFB*0+60000*0.95,'k')
plot(tFB,tFB*0+60000*1.05,'k'),legend("Vs_2","95%","105%"), xlabel("Time (h)")
subplot(3,2,4), hold on, grid minor
plot(tFB,xFB(:,4),"g")
plot(tFB,tFB*0+90000*0.95,'k')
plot(tFB,tFB*0+90000*1.05,'k'),legend("Vc_2","95%","105%"), xlabel("Time (h)")
subplot(3,2,5:6), hold on, grid minor
uTau20=-K*(xFB'-fbxEqu*ones(1,100));
plot(tFB,uTau20(1,:)),
plot(tFB,uTau20(2,:)*0),legend("Sprinkler 1","Sprinkler 2"), xlabel("Time (h)")

%% new controller
tau=log(0.05)/20;  %exp(-48*tau)=0.05 , but this one is a little bit slow
KBroken = K;
feedbackBrokenSystem=ss(A-Bbroken*KBroken,Bbroken,C,D);
t=1:100;
fbxInit=xDry(30000,:);% when the land is dryed from full state for 300 hours
fbxEqu=[60000;90000;60000;90000];
FBrokenu=KBroken*fbxEqu*ones(1,100);
[yFBroken,tFBroken,xFBroken] = lsim(feedbackBrokenSystem,FBrokenu,t,fbxInit);
% [yFBroken,tFBroken,xFBroken] = initial(feedbackSystem,xInit);
figure(6), clf
subplot(3,2,1), hold on, grid minor
plot(tFBroken,xFBroken(:,1),"c")
plot(tFBroken,tFBroken*0+60000*0.95,'k')
plot(tFBroken,tFBroken*0+60000*1.05,'k'),legend("Vs_1","95%","105%"), xlabel("Time (h)")
subplot(3,2,2), hold on, grid minor
plot(tFBroken,xFBroken(:,2),"g")
plot(tFBroken,tFBroken*0+90000*0.95,'k')
plot(tFBroken,tFBroken*0+90000*1.05,'k'),legend("Vc_1","95%","105%"), xlabel("Time (h)")
subplot(3,2,3), hold on, grid minor
plot(tFBroken,xFBroken(:,3),"c")
plot(tFBroken,tFBroken*0+60000*0.95,'k')
plot(tFBroken,tFBroken*0+60000*1.05,'k'),legend("Vs_2","95%","105%"), xlabel("Time (h)")
subplot(3,2,4), hold on, grid minor
plot(tFBroken,xFBroken(:,4),"g")
plot(tFBroken,tFBroken*0+90000*0.95,'k')
plot(tFBroken,tFBroken*0+90000*1.05,'k'),legend("Vc_2","95%","105%"), xlabel("Time (h)")
subplot(3,2,5:6), hold on, grid minor
uTau20=-K*(xFBroken'-FBrokenxEqu*ones(1,100));
plot(tFBroken,uTau20(1,:)),
plot(tFBroken,uTau20(2,:)),legend("Sprinkler 1","Sprinkler 2"), xlabel("Time (h)")

disp("not able to make a controler works better, that will distroy the first land")

