function stateDot=droneStates(X,U)
g=9.81;
mass = 0.1;
Ix = 1e-4;
Iy = 1e-4;

x=X(1);
y=X(2);
z=X(3);
xdot=X(4);
ydot=X(5);
zdot=X(6);
phi=X(7);
theta=X(8);
phidot=X(9);
thetadot=X(10);
ft=U(1);
taux=U(2);
tauy=U(3);



stateDot=[xdot;
    ydot; 
    zdot;
    -thetadot*zdot-g*sin(theta);
    phidot*zdot+g*cos(theta)*sin(phi);
    thetadot*xdot-phidot*ydot+g*cos(theta)*cos(phi)-ft/mass;
    phidot;
    thetadot;    
    taux/Ix;
    tauy/Iy];
% 
% stateDot=[xdot;
%     ydot; 
%     zdot;
%     -ft/mass*cos(phi)*sin(theta);
%     ft/mass*(sin(theta));
%     g-ft/mass*(cos(phi)*cos(theta));
%     phidot;
%     thetadot;    
%     taux/Ix;
%     tauy/Iy];