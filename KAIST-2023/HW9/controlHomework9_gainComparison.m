%10.48 b

Kp1 = 550;
Kd1 = 103;
Ki1 = 250;
c = 2;
I = 10;

sys1 = tf([Kd1,Kp1,Ki1],[I,c+Kd1,Kp1,Ki1]);

Kp2 = 35;
Kd2 = 33;
Ki2 = 10;

sys2 = tf([Kd2,Kp2,Ki2],[I,c+Kd2,Kp2,Ki2]);

step(sys1,sys2)

stepinfo(sys1)
stepinfo(sys2)

%%

sysd1 = tf([-1,0],[I,c+Kd1,Kp1,Ki1]);

sysd2 = tf([-1,0],[I,c+Kd2,Kp2,Ki2]);

bodemag(sysd1,sysd2)

%%

tau = 0.1;

sys3 = tf([100,201],[tau*125,125,100-161*tau,40]);

stepinfo(sys3)

tau = 1;

sys7 = tf([100,201],[tau*125,125,100-161*tau,40]);

figure(1)
step(sys3)
figure(2)
%step(sys4)

stepinfo(sys4)

tau = 0.3

sys7 = tf([1200,2412],[1500*tau,1500,-1932*tau,-1942]);

tau = 1

sys6 = tf(tf([1200,2412],[1500*tau,1500,-1932*tau,-1942]),1-tf([1200,2412],[1500*tau,1500,-1932*tau,-1942]));

figure(3)
step(sys7)

stepinfo(sys7)

figure(4)
step(sys6)

%%

Ka = 1;       % V/V
R = 0.3;      % Ohm
KT = 0.6;     % N*m/A
Kpot = 2;     % V/rad
I1 = 0.01;    % kg*m^2
I2 = 5e-4;    % kg*m^2
I3 = 0.2;     % kg*m^2
gear_ratio = 1/6;

tau = 0.5

% Equivalent inertia
Ie = I1 + I2 + I3;



s = tf('s');
L = 0;  % Assuming inductance is negligible initially

% Open-Loop Transfer Function without PD controller
OLTF_without_PD = (Ka * KT) / (s*(L*s + R)) * 1/(Ie * s^2) * gear_ratio * Kpot;

% Add the PD controller

KP = 5
KD = 5

PD_Controller = KP + KD * s;

% Open-Loop Transfer Function with PD controller
OLTF_with_PD = PD_Controller * OLTF_without_PD;



CLTF = OLTF_with_PD / (1 + OLTF_with_PD);


% Calculating natural frequency
wn = 2 / tau;

% Solving for KP and KD
% Here, you might need to use numerical methods or further symbolic manipulation
% depending on the complexity of the resulting equations.



% Assuming KP and KD values are calculated
PD_Controller = KP + KD * s;

% Update Closed-Loop Transfer Function with calculated KP and KD
CLTF = (PD_Controller * OLTF_without_PD) / (1 + PD_Controller * OLTF_without_PD);
sys1 = CLTF;



figure;
step(sys1);
title('Closed-Loop Step Response');


%%



syms s KP KD
Ka = 1; % V/V
R = 0.3; % Ohm
KT = 0.6; % N*m/A
Kpot = 2; % V/rad
I1 = 0.01; % kg*m^2
I2 = 5e-4; % kg*m^2
I3 = 0.2; % kg*m^2
gear_ratio = 1/6;

% Equivalent inertia
Ie = I1 + I2 + I3;

% Open-Loop Transfer Function Components
Motor_Transfer = KT / (s * R); % Assuming L = 0 for simplicity
Inertia_Transfer = 1 / (Ie * s^2);
Gear_Transfer = gear_ratio;
PD_Controller = KP + KD * s;


OLTF = PD_Controller * Ka * Motor_Transfer * Inertia_Transfer * Gear_Transfer * Kpot;

CLTF = OLTF / (1 + OLTF);


% Define the natural frequency and damping ratio equations
wn = 2 / tau; % Natural frequency
zeta = 1; % Damping ratio

% Equations based on standard second-order system characteristics
% These equations need to be formed from the characteristic polynomial of CLTF
% Example: [coeffs_CLTF, s] = coeffs(denominator(CLTF), s);
% Equate these coefficients to those of a standard second-order system
% and solve for KP and KD

% Solving the equations - this is an example, the actual equations will depend on your system
% [KP_sol, KD_sol] = solve([eqn1, eqn2], [KP, KD]);


%%

% parameters
Ka = 1;      
R = 0.3;   
KT = 0.6;    
Kpot = 2;   
I1 = 0.01;  
I2 = 5e-4;  
I3 = 0.2;   
gear_ratio = 1/6;
L = 0;

% equivalent inertia, summing assumed as 1,2,3 not dictated
Ie = I1 + I2 + I3;

% control system specifications
zeta = 1;  
tau = 0.5; 
wn = 2 / tau;

%  symbolic variables
syms KP KD s

% TF components
Motor_Transfer = KT / (L*s + R);
Inertia_Transfer = 1 / (Ie * s^2);
Gear_Transfer = gear_ratio;
PD_Controller = KP + KD * s;

% forward path eq
FP = Kpot * PD_Controller * Ka * Motor_Transfer * Inertia_Transfer * Gear_Transfer;

% cltf eq. cltf = fw / 1-LT =
CLTF = FP / (1 + FP);

% extract numerator and denominator
[Num, Den] = numden(CLTF);  % Numerator and Denominator of CLTF
Den = collect(expand(Den), s);  % Expand and collect terms in s

% extract coefficients of s
coeffs_Den = coeffs(Den, s, 'All');

% extract m c and k
k = coeffs_Den(1);
c = coeffs_Den(2);
m = coeffs_Den(3);

% solve for KP and KD
eq1 = c == 2*zeta*wn*m;
eq2 = k == wn^2*m;
sol = solve([eq1, eq2], [KP, KD]);

KP_sol = double(sol.KP)
KD_sol = double(sol.KD)

% b)
sys1 = tf ([4000*KD_sol,4000*KP_sol],[1263,4000*KD_sol,4000*KP_sol])

% c)
L2 = 0.015;

Motor_Transfer2 = KT / (L2*s + R);

PD_Controller2 = KP_sol + KD_sol * s;

FP2 = Kpot * PD_Controller2 * Ka * Motor_Transfer2 * Inertia_Transfer * Gear_Transfer;

% cltf eq. cltf = fw / 1-LT
CLTF2 = simplify(FP2 / (1 + FP2))

sys2 = tf([40,5],[4,80,40,5])

% d)

step(sys1,sys2);
xlim([0,15])
ylim([0.8,1.2])
stepinfo(sys1)
stepinfo(sys2)

%%
Kp = 0.00728;
Ki = 4.1481;
Kb = 0.199;
Ra = 0.43;
Kt = 0.14;
ce = 3.6*10^-4;
Ie = 2.08*10^-3;
La = 2.1*10^-3;
N = 1;


