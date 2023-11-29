%% Define constants
x1=pi;
x2=0;
l=1;
m=1;
b=0.1;
g=9.8;
Kp=1;
Kd=1;

thetaNoiseGain=0;
thetaDotNoiseGain=0;
%% Simulink
out = sim('as1_3.slx',10);
t=out.t;
thetaDot=out.thetaDot;
theta=out.theta;

figure(1), clf
subplot(2,1,1)
plot(t,theta)
title("PD feedback control given Kp=1, Kd=1")
ylabel("Theta (Rad)")
subplot(2,1,2)
plot(t,thetaDot)
ylabel("Theta Dot (Rad/s)")
xlabel("Time (s)")


%% Tune PD control
KdArray=[0.01, 0.1, 1, 10, 100];
KpArray=[0.01, 0.1, 1, 10, 100];
figure(2), clf
for i=1:5
    for j=1:5
        Kp=KpArray(i);
        Kd=KdArray(j);
        out = sim('as1_3.slx',10);
        t=out.t;
        thetaDot=out.thetaDot;
        theta=out.theta;

        subplot(5,5,i*5-5+j)
        plot(t,theta)
       
        if j==1
            ylabel(sprintf("Kp=%.2f\nTheta (Rad)",Kp))
        end

        if i==5
            xlabel(sprintf("Kd=%.2f\nTime (s)",Kd))
        end

        ylim([-5,5])
    end
end

%% noise
Kp=1;
Kd=1;
noiseArray=[0, 0.01, 0.1, 1, 10];

figure(3), clf
figure(4), clf
for i=1:length(noiseArray)
    thetaNoiseGain=noiseArray(i);
    thetaDotNoiseGain=noiseArray(i);
    out = sim('as1_3.slx',10);
    t=out.t;
    thetaDot=out.thetaDot;
    theta=out.theta;
    thetaObserved=out.thetaOb;
    
    figure(3)
    subplot(length(noiseArray),1,i), hold on
    
    plot(t, thetaObserved)
    plot(t,theta)
    
    legend("Observed Theta","True Theta")
    ylabel(sprintf("Noise %.0f%%\nTheta (Rad)",thetaNoiseGain*100))

    ylim([-2,4])

    figure(4), hold on
    plot(t,theta)
    
end
figure(4)
legend("Noise 0%","Noise 10%","Noise 100%","Noise 1000%","Noise 10000%")

figure(3)
xlabel("Time (s)")

%% PD control Under Noise
KdArray=[0.1, 1, 10];
KpArray=[0.1, 1, 10];

figure(5), clf
for i=1:length(KdArray)
    for j=1:length(KpArray)
        Kp=KpArray(i);
        Kd=KdArray(j);

        thetaNoiseGain=0;
        thetaDotNoiseGain=0;

        out = sim('as1_3.slx',10);
        t=out.t;
        thetaDot=out.thetaDot;
        theta=out.theta;

        thetaNoiseGain=3;
        thetaDotNoiseGain=3;
        
        out = sim('as1_3.slx',10);
        thetaUnderNoise=out.theta;

        subplot(length(KdArray),length(KpArray),i*length(KdArray)-length(KdArray)+j), hold on
        plot(t,theta)
        plot(t,thetaUnderNoise)
       
        if j==1
            ylabel(sprintf("Kp=%.2f\nTheta (Rad)",Kp))
        end

        if i==length(KdArray)
            xlabel(sprintf("Kd=%.2f\nTime (s)",Kd))
        end

        ylim([-5,5])
    end
end