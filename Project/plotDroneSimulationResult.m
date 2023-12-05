function plotDroneSimulationResult(out)

out.simuRealStates=reshape(out.simuRealStates,10,[])';
out.simuRealU=reshape(out.simuRealU,3,[])';
%% postion
subplot(5,3,1), hold on
plot(out.simuT,out.simuLinearStates(:,1))
plot(out.simuT,out.simuRealStates(:,1))
% legend("Linear","Real")
ylabel("x position"),grid minor

subplot(5,3,2), hold on
plot(out.simuT,out.simuLinearStates(:,2))
plot(out.simuT,out.simuRealStates(:,2))
% legend("Linear","Real")
ylabel("y position"),grid minor

subplot(5,3,3), hold on
plot(out.simuT,out.simuLinearStates(:,3),'.')
plot(out.simuT,out.simuRealStates(:,3))
% legend("Linear","Real")
ylabel("z position"),grid minor

%% velocity
subplot(5,3,4), hold on
plot(out.simuT,out.simuLinearStates(:,4))
plot(out.simuT,out.simuRealStates(:,4))
% legend("Linear","Real")
ylabel("x velocity"),grid minor

subplot(5,3,5), hold on
plot(out.simuT,out.simuLinearStates(:,5))
plot(out.simuT,out.simuRealStates(:,5))
% legend("Linear","Real")
ylabel("y velocity"),grid minor

subplot(5,3,6), hold on
plot(out.simuT,out.simuLinearStates(:,6),'.')
plot(out.simuT,out.simuRealStates(:,6))
% legend("Linear","Real")
ylabel("z velocity"),grid minor

%% angles
subplot(5,3,7), hold on
plot(out.simuT,out.simuLinearStates(:,7))
plot(out.simuT,out.simuRealStates(:,7))
% legend("Linear","Real")
ylabel("x angle"),grid minor

subplot(5,3,8), hold on
plot(out.simuT,out.simuLinearStates(:,8))
plot(out.simuT,out.simuRealStates(:,8))
% legend("Linear","Real")
ylabel("y angle"),grid minor

subplot(5,3,10), hold on
plot(out.simuT,out.simuLinearStates(:,9))
plot(out.simuT,out.simuRealStates(:,9))
% legend("Linear","Real")
ylabel("x angular velocity"),grid minor

subplot(5,3,11), hold on
plot(out.simuT,out.simuLinearStates(:,10))
plot(out.simuT,out.simuRealStates(:,10))
% legend("Linear","Real")
ylabel("y angular velocity"),grid minor
%% control signals
subplot(5,3,13:15), hold on
plot(out.simuT,out.simuRealU(:,1))
plot(out.simuT,out.simuLinearU(:,1))
legend("Linear","Real")
ylabel("Fm"),grid minor

subplot(5,3,9), hold on
plot(out.simuT,out.simuRealU(:,2))
plot(out.simuT,out.simuLinearU(:,2))
% legend("Linear","Real")
ylabel("Tau_x"),grid minor

subplot(5,3,12), hold on
plot(out.simuT,out.simuRealU(:,3))
plot(out.simuT,out.simuLinearU(:,3))
% legend("Linear","Real")
ylabel("Tau_y"),grid minor

subplot(5,3,13:15)
end