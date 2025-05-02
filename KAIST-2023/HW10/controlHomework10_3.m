% Define the transfer function G_p(s)
kp = 0.08867
numerator = [-2*kp 1*kp 26*kp];
denominator = [1 5 6 0];
G_p = tf(numerator, denominator);
step(G_p)


% Plot the root locus
figure;
rlocus(G_p);
title('Root Locus of G_p(s)');

% Determine the range of Kp for stability
[rloc, k] = rlocfind(G_p);
% rloc gives the location of poles for the selected point on the root locus
% k gives the corresponding gain (Kp) value

rlocus(tf(1,denominator))

step(tf)