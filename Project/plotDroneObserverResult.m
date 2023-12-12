function plotDroneObserverResult(out)

out.simuReal2bEstStates=reshape(out.simuReal2bEstStates,10,[])';
out.simuEstStates=reshape(out.simuEstStates,10,[])';
out.simuReal2bEstOutput=reshape(out.simuReal2bEstOutput,7,[])';
% out.simuRealU=reshape(out.simuRealU,3,[])';
%% postion
subplot(5,3,1), hold on
plot(out.simuT,out.simuEstStates(:,1))
plot(out.simuT,out.simuReal2bEstStates(:,1))
legend("Estimated","Real")
ylabel("x position"),grid minor

subplot(5,3,2), hold on
plot(out.simuT,out.simuEstStates(:,2))
plot(out.simuT,out.simuReal2bEstStates(:,2))
% legend("Linear","Real")
ylabel("y position"),grid minor

subplot(5,3,3), hold on
plot(out.simuT,out.simuEstStates(:,3))
plot(out.simuT,out.simuReal2bEstStates(:,3))
% legend("Linear","Real")
ylabel("z position"),grid minor

%% velocity
subplot(5,3,4), hold on
plot(out.simuT,out.simuEstStates(:,4))
plot(out.simuT,out.simuReal2bEstStates(:,4))
% legend("Linear","Real")
ylabel("x velocity"),grid minor

subplot(5,3,5), hold on
plot(out.simuT,out.simuEstStates(:,5))
plot(out.simuT,out.simuReal2bEstStates(:,5))
% legend("Linear","Real")
ylabel("y velocity"),grid minor

subplot(5,3,6), hold on
plot(out.simuT,out.simuEstStates(:,6))
plot(out.simuT,out.simuReal2bEstStates(:,6))
% legend("Linear","Real")
ylabel("z velocity"),grid minor

%% angles
subplot(5,3,7), hold on
plot(out.simuT,out.simuEstStates(:,7))
plot(out.simuT,out.simuReal2bEstStates(:,7))
% legend("Linear","Real")
ylabel("x angle"),grid minor

subplot(5,3,8), hold on
plot(out.simuT,out.simuEstStates(:,8))
plot(out.simuT,out.simuReal2bEstStates(:,8))
% legend("Linear","Real")
ylabel("y angle"),grid minor

subplot(5,3,10), hold on
plot(out.simuT,out.simuEstStates(:,9))
plot(out.simuT,out.simuReal2bEstStates(:,9))
% legend("Linear","Real")
ylabel("x angular velocity"),grid minor

subplot(5,3,11), hold on
plot(out.simuT,out.simuEstStates(:,10))
plot(out.simuT,out.simuReal2bEstStates(:,10))
% legend("Linear","Real")
ylabel("y angular velocity"),grid minor
%% Tracking 
subplot(5,3,13), hold on
plot(out.simuT,out.simuReal2bEstOutput(:,3))
plot(out.simuT,out.simuTargetZ,'k','LineWidth',1)
legend("Real","Target")
ylabel("Z Position"),grid minor

subplot(5,3,14), hold on
plot(out.simuT,out.simuReal2bEstOutput(:,6))
plot(out.simuT,-out.simuTargetX,'k','LineWidth',1)
% legend("Real","Target")
ylabel("X Position"),grid minor

subplot(5,3,15), hold on
plot(out.simuT,out.simuReal2bEstOutput(:,7))
plot(out.simuT,-out.simuTargetY,'k','LineWidth',1)
% legend("Real","Target")
ylabel("Y Position"),grid minor

%%
subplot(5,3,9), hold on
% plot(out.simuT,out.simuRealU(:,2))
% plot(out.simuT,out.simuLinearU(:,2))
% legend("Linear","Real")
ylabel("Tau_x"),grid minor

subplot(5,3,12), hold on
% plot(out.simuT,out.simuRealU(:,3))
% plot(out.simuT,out.simuLinearU(:,3))
% legend("Linear","Real")
ylabel("Tau_y"),grid minor

end