% Define the transfer function Gp(s)
numerator = [-2 1 26];
denominator = [1 5 6 0];
Gp = tf(numerator, denominator);

% Find the poles and zeros of the open-loop system
[zeros_Gp, poles_Gp] = pzmap(Gp);

% Define the characteristic equation of the closed-loop system
syms s Kp
char_eq = 1 + Kp * (-2*s^2 + s + 26) / (s*(s + 2)*(s + 3));

% Differentiate the characteristic equation w.r.t s and solve for s
dchar_eq = diff(char_eq, s)
s_breakaway = double(solve(dchar_eq == 0, s))

% Filter out the breakaway points that lie on the real axis and between open-loop poles
s_breakaway = s_breakaway(imag(s_breakaway) == 0);  % Keep only real roots
s_breakaway = s_breakaway(s_breakaway > min(poles_Gp) & s_breakaway < max(poles_Gp));

% Calculate Kp at these breakaway points
Kp_at_breakaway = double(subs(Kp, solve(subs(char_eq, s, s_breakaway(1)) == 0, Kp)));

% Display results
disp(['Breakaway points (s): ', num2str(s_breakaway.')]);
disp(['Corresponding Kp values: ', num2str(Kp_at_breakaway.')]);
