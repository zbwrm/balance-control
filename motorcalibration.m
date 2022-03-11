clear all;
motordata = readmatrix("motortest2.csv");

sample_rate = 100; % Hertz
step = (1:80)' - 1;
timestep = (step ./ sample_rate);
leftmotor = motordata(:,1);
rightmotor = motordata(:,2);
expfittype = fittype("a-a*exp(-b*x)");

figure();
title("Motor Calibration")
subplot(1,2,1);
plot(timestep, leftmotor(1:80,:),'.'); hold on
leftfit = fit(timestep, leftmotor(1:80), expfittype);
plot(leftfit); text(0.45,0.04,sprintf('a=%.3f, b=%.4f', leftfit.a, 1/leftfit.b)); 
title('Left motor'); ylabel("Motor Speed"); xlabel("Time (s)")

subplot(1,2,2);
plot(timestep, rightmotor(1:80,:),'.'); hold on
rightfit = fit(timestep, rightmotor(1:80), expfittype);
plot(rightfit); text(0.45,0.04,sprintf('a=%.3f, b=%.4f', rightfit.a, 1/rightfit.b)); 
title("Right Motor"); ylabel("Motor Speed"); xlabel("Time (s)")