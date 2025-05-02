% Problem 2
g1 = tf(6,[1,0])
g2 = tf(1,[4,1])
g3 = tf(1,[3,2])
g4 = tf(10)
g5 = inv(g1)*10+1

sys = feedback(series(g1,feedback(series(g2,g3),g4)),1)
step(sys)

sys2 = -feedback(series(series(series(g3,-g5),g1),g2),1,+1)
step(sys2)

% Problem 3
% params
m = 100
c = 600
k1 = 8000
k2 = 24000
tau = 0.01
t = 0:0.01:10
y = 1 - exp(-t/tau)

springDamper = tf([k2,0],[m,c,k1])
step(springDamper)



mastLifter = tf()