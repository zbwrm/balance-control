clear all;
motordata = csvread("motortest2.csv");

sample_rate = 100; % Hertz
step = (1:80)' - 1;
timestep = (step ./ sample_rate);
leftmotor = motordata(:,1);
rightmotor = motordata(:,2);

plot(timestep, leftmotor(1:80))
leftfit = fit(timestep, leftmotor(1:80), 'exp1');


% lcroppeddata = leftmotor(1:80,:);
% rcroppeddata = rightmotor(1:80,:);
% step_for_cftool = (1:length(lcroppeddata))';