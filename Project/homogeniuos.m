
%% Homogenous Response
stepVal=[0,0,0];
duration=10;
x0=randn(n,1).*[1,1,1,1,1,1,0.1,0.1,0,0]'*0.1; %Initial Condition
try
   out = sim('drone_5DOF.slx',duration);
catch exception
   disp(exception)
   return
end
figure(5)
plotDroneSimulationResult(out)
xlabel("Time (t) Homogenous Response")
%% Step Response: Step in Pulling Force
stepVal=[1,0,0];
duration=10;
x0=zeros(10,1); %Initial Condition
try
   out = sim('drone_5DOF.slx',duration);
catch exception
   disp(exception)
   return
end
figure(6)
plotDroneSimulationResult(out)
xlabel("Time (t) Step in Pulling Force")
%% Step Response: Step in TauX
stepVal=[0,1,0];
duration=10;
x0=zeros(10,1); %Initial Condition
try
   out = sim('drone_5DOF.slx',duration);
catch exception
   disp(exception)
   return
end
figure(7)
plotDroneSimulationResult(out)
xlabel("Time (t) Step in Tau_X")
%% Step Response: Step in TauY
stepVal=[0,0,1];
duration=10;
x0=zeros(10,1); %Initial Condition
try
   out = sim('drone_5DOF.slx',duration);
catch exception
   disp(exception)
   return
end
figure(8)
plotDroneSimulationResult(out)
xlabel("Time (t) Step in Tau_Y")
%% Homogenous Response with Observer
stepVal=[0,0,0];
duration=3;
x0=randn(n,1).*[1,1,1,1,1,1,0.1,0.1,0,0]'*0.1; %Initial Condition
try
   out = sim('drone_5DOF.slx',duration);
catch exception
   disp(exception)
   return
end
figure(5)
plotDroneObserverResult(out)
xlabel("Time (t) Homogenous Response")