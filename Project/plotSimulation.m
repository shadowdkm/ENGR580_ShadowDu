close all
duration=30;
x0=randn(n,1).*[1,1,1,1,1,1,0.1,0.1,0,0]'*0.001; %Initial Condition
try
   out = sim('drone_5DOF.slx',duration);
catch exception
   disp(exception)
   return
end

%%
size(out.simuT)
size(out.simuLinearStates)
size(out.simuRealStates)
out.simuRealStates=reshape(out.simuRealStates,10,[])';
out.simuRealU=reshape(out.simuRealU,3,[])';



figure, clf
subplot(2,1,1), hold on
plot(out.simuT,out.simuLinearStates(:,3))
plot(out.simuT,out.simuRealStates(:,3))
legend("Linear","Real")
ylabel("z position")
subplot(2,1,2), hold on
plot(out.simuT,out.simuLinearStates(:,6))
plot(out.simuT,out.simuRealStates(:,6))
legend("Linear","Real")
ylabel("z velocity")
xlabel("Time (s)")

figure, clf
subplot(2,1,1), hold on
plot(out.simuT,out.simuLinearStates(:,7))
plot(out.simuT,out.simuRealStates(:,7))
legend("Linear","Real")
ylabel("Phi")
subplot(2,1,2), hold on
plot(out.simuT,out.simuLinearStates(:,8))
plot(out.simuT,out.simuRealStates(:,8))
legend("Linear","Real")
ylabel("Theta")
xlabel("Time (s)")

figure, hold on
plot(out.simuT,out.simuLinearU(:,2)')
plot(out.simuT,out.simuRealU(:,2)')
legend("Linear","Real")
ylabel("TauX")
xlabel("Time (s)")

figure, hold on
plot(out.simuT,out.simuLinearU(:,3)')
plot(out.simuT,out.simuRealU(:,3)')
legend("Linear","Real")
ylabel("TauY")
xlabel("Time (s)")

out.simuRealStates(2,:)
out.simuLinearStates(2,:)

figure, hold on
plot(out.simuRealU(:,1))
plot(out.simuLinearU(:,1))
legend("RealU","LinearU")

%%
clc
x0=randn(n,1)/100;
disp("A*x0")
disp(droneStates(x0,[0,0,0])')
disp((A*x0)')

disp("B*[1;0.001;0.001]")
disp(droneStates([0,0,0,0,0,0,0,0,0,0],[1,0.001,0.001])')
disp((B*[1;0.001;0.001])')

disp("Ax+Bu")
disp(droneStates(x0,[1,0.001,0.001])')
disp((A*x0)'+(B*[1;0.001;0.001])')


