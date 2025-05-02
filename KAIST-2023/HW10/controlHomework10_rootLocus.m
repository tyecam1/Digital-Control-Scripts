% params
Kb = 0.1;
Kt = 0.1;
I = 12 * 10^-5;
Ra = 2;
La = 3 * 10^-3;
c_values = linspace(0.01, 2, 100);

time_constants = zeros(length(c_values), 1);
c = 1;

% transfer function
num = Kb*Kt;
den = [La*I, Ra*La + c*La, c*Ra + Kb*Kt];
sys = tf(num, den);

rlocus(sys);
grid on;

