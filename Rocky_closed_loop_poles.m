% defind syms
% values
syms s a b l g Kp Ki Jp Ji Ci

% TF from velocity to angle of inverted pendlum
Hvtheta = -(s/l)/(s^2-(g/l));
% TF of angle controller
K = Kp + Ki/s;
% TF of the controller around the motor
J = Jp + Ji/s + Ci/(s^2);
% TF of motor
M = (a*b)/(s+a);
% TF of motor and feedback controller around it
Md = M/(1+M*J);

% Total TF function
Htot = 1/(1-Hvtheta*Md*K);

% system parameters
g = 9.81;
a = (0.0325+0.0371)/2;
b = (0.69+0.654)/2;
wn = 5;
l = g/(wn^2);

% sub in parameter values
Htot_subbed = simplify(subs(Htot));

% define target poles
angle1 = 44;
angle2 = 35;

real_offset = 0.0;

p1 = wn*(-cosd(angle1) + i*sind(angle1));
p2 = wn*(-cosd(angle1) - i*sind(angle1));
p3 = wn*(-cosd(angle2) - real_offset+ i*sind(angle2));
p4 = wn*(-cosd(angle2) - real_offset - i*sind(angle2));
p5 = -wn;

%{
p1 = wn*(-cosd(angle1) + sind(angle1));
p2 = wn*(-cosd(angle1) - sind(angle1));
p1 = wn*(-cosd(angle2) - real_offset + sind(angle2));
p2 = wn*(-cosd(angle2) - real_offset - sind(angle2));
p5 = -wn;
%}
% define target characterisitic polynomial
%{
p1 = -4 + 2*i;
p2 = -4 - 2*i;
p3 = -4 + i;
p4 = -4 - i;
p5 = -5;
%}
char_poly = (s-p1)*(s-p2)*(s-p3)*(s-p4)*(s-p5);

% find coeffs of char polynomial denom
coeffs_char = coeffs(char_poly,s);

% get denominator of Htot_subbed
[~, denom] = numden(Htot_subbed);

% find coefficients of the denom
coeffs_denom = coeffs(denom,s);

% divide out coeff of highest power term
coeffs_denom = coeffs_denom/coeffs_denom(end);

% solve the system of equations setting the coefficients of the polunomial
% in the target to the actual polynomial
solutions = solve(coeffs_denom == coeffs_char,[Kp,Ki,Jp,Ji,Ci]);

% display the solutions as doubles
format shortG
Kp = double(solutions.Kp)
Ki = double(solutions.Ki)
Jp = double(solutions.Jp)
Ji = double(solutions.Ji)
Ci = double(solutions.Ci)
kmotor = b;
tau = 1/a;