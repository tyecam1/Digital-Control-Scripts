
m1 = 30
m2 = 70
k1 = 3*10^4
k2 = 6*10^4

X1F = tf([m2,0,k2],[m1*m2,0,m1*k2+m2*k2,0,0])+tf(1,k1)
X2F = tf(k1,[m1*m2,0,m1*k2+m2*k2,0])+tf(1,[m2,0,k2])

figure(1)
step(X1F)
xlim([0 100])
hold on
grid on
figure(2)
step(X2F)
xlim([0 100])
hold on
grid on


