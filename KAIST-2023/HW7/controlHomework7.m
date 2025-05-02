



% 8.44

% a
sys = tf(1,[1,22,113,110])
stepinfo(sys,'RiseTimeLimits',[0,1])
step(sys)

% b
poles = pole(sys);

dominantPole = poles(3);

wn = abs(poles(3));
zeta = -dominantPole/abs(dominantPole);
% zeta = 1, critically damped

Mp = exp((-pi*zeta)/sqrt(1-zeta^2)) * 100
peakTime = pi/(wn*sqrt(1-zeta^2))

% Very inaccurate as zeta != 0.5
riseTime = 1.8 / wn




% 8.50
sim('CamSystem')
plot(ans.tout,ans.CamSystem(:)),legend('x'),ylim([-0.5,3]),xlabel('Time'),ylabel('x')
