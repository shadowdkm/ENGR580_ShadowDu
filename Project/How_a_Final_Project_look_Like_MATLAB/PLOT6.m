%clc;clear;
close all;
%load('Car_data');
index =  1;
t=(1:numel(System.state(2,:)))*T
Fontsize=11;
rng(0);color=rand(1,3)

%% Outputs :angles

subplot(3,1,1)
plot(t,System.state(output_index(1),:))
xlabel('$$Time [sec]$$','Interpreter','latex');ylabel('$$\phi [rad]$$','Interpreter','latex');
grid
ylim([-0.21 0.21])
set(gca, 'FontSize', Fontsize);
set(gca, 'FontName', 'Times New Roman');

subplot(3,1,2)
plot(t,System.state(output_index(2),:))
xlabel('$$Time [sec]$$','Interpreter','latex');ylabel('$$\theta [rad]$$','Interpreter','latex');
grid
ylim([-0.21 0.21])
set(gca, 'FontSize', Fontsize);
set(gca, 'FontName', 'Times New Roman');

subplot(3,1,3)
plot(t,System.state(output_index(3),:))
xlabel('$$Time [sec]$$','Interpreter','latex');ylabel('$$\psi [rad]$$','Interpreter','latex');
grid
ylim([-0.21 0.21])
set(gca, 'FontSize', Fontsize);
set(gca, 'FontName', 'Times New Roman');

exportgraphics(gcf,'PIC\angles.pdf','ContentType','vector')
saveas(gcf,'PIC\angles.svg')
%% Outputs :positions
figure
subplot(3,1,1)
plot(t,System.state(output_index(4),:))
xlabel('$$Time [sec]$$','Interpreter','latex');ylabel('$$x [m]$$','Interpreter','latex');
grid
ylim([-1.1 1.1])
set(gca, 'FontSize', Fontsize);
set(gca, 'FontName', 'Times New Roman');

subplot(3,1,2)
plot(t,System.state(output_index(5),:))
xlabel('$$Time [sec]$$','Interpreter','latex');ylabel('$$y [m]$$','Interpreter','latex');
grid
ylim([-1.1 1.1])
set(gca, 'FontSize', Fontsize);
set(gca, 'FontName', 'Times New Roman');

subplot(3,1,3)
plot(t,System.state(output_index(6),:))
xlabel('$$Time [sec]$$','Interpreter','latex');ylabel('$$z [m]$$','Interpreter','latex');
grid
ylim([-1.1 1.1])

set(gca, 'FontSize', Fontsize);
set(gca, 'FontName', 'Times New Roman');

exportgraphics(gcf,'PIC\position.pdf','ContentType','vector')
saveas(gcf,'PIC\position.svg')

%% Inputs
figure
subplot(4,1,1)
hold on
plot(t,System.input_learn(1,:))
plot(t,System.input_safe(1,:))
xlabel('$$Time [sec]$$','Interpreter','latex');ylabel('$$u_1 [N]$$','Interpreter','latex');
grid;box
ylim([-1.1 1.1])
set(gca, 'FontSize', Fontsize);
set(gca, 'FontName', 'Times New Roman');
legend('Learning','Safe')

h = legend('Location','bestoutside'); 

subplot(4,1,2)
hold on
plot(t,System.input_learn(2,:))
plot(t,System.input_safe(2,:))
xlabel('$$Time [sec]$$','Interpreter','latex');ylabel('$$u_2 [N.m]$$','Interpreter','latex');
grid;box
ylim([-0.11 0.11])
set(gca, 'FontSize', Fontsize);
set(gca, 'FontName', 'Times New Roman');

subplot(4,1,3)
hold on
plot(t,System.input_learn(3,:))
plot(t,System.input_safe(3,:))
xlabel('$$Time [sec]$$','Interpreter','latex');ylabel('$$u_3 [N.m]$$','Interpreter','latex');
grid;box
ylim([-0.11 0.11])
set(gca, 'FontSize', Fontsize);
set(gca, 'FontName', 'Times New Roman');


subplot(4,1,4)
hold on
plot(t,System.input_learn(4,:))
plot(t,System.input_safe(4,:))
xlabel('$$Time [sec]$$','Interpreter','latex');ylabel('$$u_4 [N.m]$$','Interpreter','latex');
grid;box
ylim([-0.11 0.11])
set(gca, 'FontSize', Fontsize);
set(gca, 'FontName', 'Times New Roman');

exportgraphics(gcf,'PIC\inputs.pdf','ContentType','vector')
saveas(gcf,'PIC\inputs.svg')

