% zeros and poles
zeros = [0, -0.5, -1.5];
poles = [-2.5, -3.5 - 3.5i, -3.5 + 3.5i];

% zero pole gain tf func
G = zpk(zeros, poles, 1);

% plot the root locus
figure;
rlocus(G);
title('Root Locus Plot');
grid on;

% tau = -1 / real pole => pole = -2
[K, poles] = rlocfind(G, -2)
