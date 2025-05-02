Kb = 0.2;
KT = Kb;
cm = 3*10^-4;
Ra = 0.8;
La = 4*10^-3;
Im = 4*10^-4;
N = 3;
IL = 10^-3;
TL = 0.04;
cL = 1.8*10^-3;
V = 20;

I = Im +IL/N^2;
c = cm + cL/N^2;
A = [-Ra/La,-Kb/La;KT/I,-c/I;];
B = [1/La,0;0, -1/(N*I)];
C = [1,0;0,1];
D = [0,0;0,0];
sysmotor = ss(A,B,C,D);

[y, t] = step(sysmotor);
subplot(2,1,1),plot(t,10*y(:,1)),...
xlabel('t (s)'),ylabel('Current (A)')
subplot(2,1,2),plot(t,10*y(:,2)),...
xlabel('t (s)'),ylabel('Speed (rad/s)')

peak_ia = max(10*y(:,1))