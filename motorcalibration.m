clear all;
motordata = csvread("motortest2.csv");

leftmotor = motordata(:,1);
rightmotor = motordata(:,2);

lcroppeddata = leftmotor(1:80,:);
rcroppeddata = rightmotor(1:80,:);
step_for_cftool = (1:length(lcroppeddata))';