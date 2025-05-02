% Define the system parameters
numerator = [-2 1 26];
denominator = [1 5 6 0];
Gp = tf(numerator, denominator);
rlocus(Gp)
% Define symbolic variables
syms s Kp

% Substitute the desired pole into the characteristic equation
%desired_pole = -omega_n;  % Replace omega_n with a specific value or range
char_eq = 1 + Kp * (-2*s^2 + s + 26) / (s*(s + 2)*(s + 3)) == 0;
%Kp_solution = solve(subs(char_eq, s, desired_pole), Kp);

% Calculate Kp
Kp_value = double(Kp_solution)

