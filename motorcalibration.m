clear all;
motordata = csvread("motortest2.csv");

sample_rate = 100; % Hertz
step = (1:80)' - 1;
timestep = (step ./ sample_rate);
leftmotor = motordata(1:80,1);
rightmotor = motordata(1:80,2);

x = linspace(0,0.8);
k_left = 0.003068;
tau_left = 0.05114;
left_fit = 300*k_left*(1-exp(-x/tau_left));

k_right = 0.002931;
tau_right = 0.03569;
right_fit = 300*k_right*(1-exp(-x/tau_right));

hold on
plot(x,left_fit, 'b')
plot(x,right_fit, 'r')
scatter(timestep, leftmotor(1:80), '.', 'b')
scatter(timestep, rightmotor(1:80), '.', 'r')
xlabel('time')
ylabel('speed')
legend('left', 'right', 'location', 'southeast')
hold off




%leftfit = fit(timestep, leftmotor(1:80), 'exp1');
%plot(leftfit)

% lcroppeddata = leftmotor(1:80,:);
% rcroppeddata = rightmotor(1:80,:);
% step_for_cftool = (1:length(lcroppeddata))';

%leftmotor parameters
% General model Exp2:
%      f(x) = a*exp(b*x) + c*exp(d*x)
% Coefficients (with 95% confidence bounds):
%        a =      0.9098  (0.8964, 0.9233)
%        b =     0.02332  (-0.00475, 0.05138)
%        c =     -0.9107  (-0.9377, -0.8838)
%        d =      -20.06  (-21.26, -18.86)
