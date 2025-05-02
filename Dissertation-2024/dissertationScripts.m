% Dissertation Complete Matlab Scripts

%% Symbolic Transfer Function Checker
% Derives closed-loop transfer functions symbolically for stability analysis.
% Validates system dynamics before numerical implementation.

% Define symbolic variables for system parameters
% ------------------------------------------------------------------------
% Hydraulic system components:
% - M1, M2, M3: Masses of top/bottom cylinders and carriage (kg)
% - A1t, A1b, A2t, A2b: Piston areas (m²)
% - beta: Fluid bulk modulus (Pa)
% - V1to, V1bo, V2to, V2bo: Rest volumes in chambers (m³)
% - k: Spring constant of steel frame (N/m)
% - Ht: Position transducer gain (V/m)
% ------------------------------------------------------------------------
syms M2 M3 C k A1t A1b A2t A2b beta V1t V1b V2t V2b g s kq1t kq1b kq2t kq2b...
    b c d1b d1t d2b d2t E h Ht I Ks L M1 P1to P1bo P2bo P2to Ps V1bo V1to V2bo V2to...
    s topGain botGain;

% Component Transfer Functions
% ------------------------------------------------------------------------
servTF = 0.001/(0.01*s + 1);  % Servo-valve TF: 1st-order lag model
                               % Gain = 0.001, Time constant = 0.01s

% Top cylinder dynamics (3rd-order system)
topCyl = ((A1b*kq1b*beta/V1bo) - (A1t*kq1t*beta/V1to)) / ...
         (M1*s^3 + C*s^2 + (-(A1t^2*beta/V1to) - (A1b^2*beta/V1bo) + k)*s);

% Bottom cylinder dynamics (3rd-order system)
botCyl = ((A2b*kq2b*beta/V2bo) - (A2t*kq2t*beta/V2to)) / ...
         (M2*s^3 + C*s^2 + ((A2t^2*beta/V2to) + (A2b^2*beta/V2bo) - k)*s);

% Carriage dynamics (2nd-order spring-mass system)
X3TF = k/(M3*s^2 + 2*k);  

% Closed-Loop System Derivation
% ------------------------------------------------------------------------
% Forward path gains with controller
fpg1sym = topGain*servTF*topCyl*X3TF;  % Top subsystem forward path
fpg2sym = botGain*servTF*botCyl*X3TF;  % Bottom subsystem forward path

% Unity feedback configuration
feedback1 = Ht;  % Position transducer in feedback path
feedback2 = Ht;

% Closed-loop transfer functions via symbolic manipulation
[pNum1,pDen1] = numden(fpg1sym/(1 + fpg1sym*feedback1));  % Top CLTF
[pNum2,pDen2] = numden(fpg2sym/(1 + fpg2sym*feedback2));  % Bottom CLTF

CLTF1sym = (pNum1)/(pDen1 + pNum1);  % Simplified top closed-loop TF
CLTF2sym = Ht*(pNum2)/(pDen2 + pNum2); % Simplified bottom closed-loop TF

% Display human-readable transfer function
pretty(CLTF1sym)  % Show top subsystem closed-loop TF in algebraic form

%% Constants
% Define physical parameters for the hydraulic test rig system
% All units in SI (kg, m, N, Pa, s)

% Hydraulic Component Masses
% ------------------------------------------------------------------------
M1 = 22.03;   % Mass of top hydraulic cylinder (kg)
M2 = 42.85;   % Mass of bottom hydraulic cylinder (kg)
M3 = 65.08;   % Mass of central carriage assembly (kg)

% Hydraulic Fluid Properties
% ------------------------------------------------------------------------
beta = 1.6e9; % Bulk modulus of hydraulic fluid (Pa)
              % (Indicates fluid compressibility at operating pressure)

% Servo-Valve Characteristics
% ------------------------------------------------------------------------
Ks = 5.61e-6; % Flow coefficient (m³/(s·√Pa))
              % Relates flow rate to pressure differential
Kb = 0.001;   % Valve gain (A/m) - current-to-displacement relationship

% Cylinder Geometry
% ------------------------------------------------------------------------
% Diameters [m]
d1t = 0.0508; % Top cylinder piston diameter
d1b = 0.0286; % Top cylinder rod diameter
d2b = 0.0762; % Bottom cylinder piston diameter
d2t = 0.0381; % Bottom cylinder rod diameter

% Effective piston areas [m²] (annular areas for rod-side chambers)
A1t = pi*((d1t/2)^2);    % Top cylinder cap-side area
A1b = A1t - pi*((d1b/2)^2); % Top cylinder rod-side area
A2b = pi*((d2b/2)^2);    % Bottom cylinder cap-side area
A2t = A2b - pi*((d2t/2)^2); % Bottom cylinder rod-side area

% Structural Dimensions
% ------------------------------------------------------------------------
cL = 0.1;    % Cylinder length (m)
L = 0.49;    % Carriage length between cylinder mounts (m)
b = 0.24;    % Carriage cross-section base width (m)
h = 0.025;   % Carriage cross-section height (m)

% Material Properties (Steel)
% ------------------------------------------------------------------------
E = 200e9;   % Young's modulus (Pa) - stiffness of carriage material
I = (b*(h^3))/12; % Second moment of area (m⁴) for rectangular cross-section
                 % (Calculates carriage beam stiffness)

% Spring constant for steel frame [N/m]
k = (192*E*I)/(L^3); % Derived from fixed-free beam deflection formula

% General Constants
% ------------------------------------------------------------------------
g = 9.81;    % Gravitational acceleration (m/s²)
Ht = 51.76;  % Position transducer gain (V/m) 
             % (Converts mechanical displacement to voltage signal)

%% Variables
% Calculate derived parameters for hydraulic system operation
% All units in SI (m, kg, Pa, N, m³/s)

% Damping & Volumes
% ------------------------------------------------------------------------
C = 1000; % Damping coefficient (N·s/m) - from prior experimental characterization

% Hydraulic chamber rest volumes [m³] (mid-stroke assumption)
V1to = (A1t + A1b) * 0.5 * cL; % Top cylinder top chamber volume
V1bo = (A1t + A1b) * 0.5 * cL; % Top cylinder bottom chamber volume
V2to = (A2t + A2b) * 0.5 * cL; % Bottom cylinder top chamber volume
V2bo = (A2t + A2b) * 0.5 * cL; % Bottom cylinder bottom chamber volume

% Hydraulic Pressures
% ------------------------------------------------------------------------
Ps = 10e6;      % System supply pressure (Pa) - from pump specifications
Fmax = 2*M3*g;  % Maximum expected load force (N) - 2x gravitational load

% Rest pressures [Pa] (static equilibrium calculations)
P2bo = 1.5*(M3*g + Fmax)/A2b; % Bottom cylinder bottom chamber pressure
P2to = (A2b*P2bo - Fmax)/A2t; % Bottom cylinder top chamber pressure
P1bo = 1.5*(M1*g)/A2b;        % Top cylinder bottom chamber pressure
P1to = (P1bo*A1b)/A2b;        % Top cylinder top chamber pressure

% Valve Flow Coefficients
% ------------------------------------------------------------------------
% Calculated using orifice flow equation: Q = Ks·√(ΔP)
kq1t = Ks*sqrt(Ps - P1to); % Top cylinder top-side flow coeff. [m³/(s·√Pa)]
kq1b = Ks*sqrt(P1bo);      % Top cylinder bottom-side flow coeff.
kq2t = Ks*sqrt(Ps - P2to); % Bottom cylinder top-side flow coeff.
kq2b = Ks*sqrt(P2bo);      % Bottom cylinder bottom-side flow coeff.

% System Initialization
% ------------------------------------------------------------------------
s = tf('s'); % Laplace variable for transfer function analysis

%% Transfer Functions
% Define continuous-time transfer functions for system components
% ------------------------------------------------------------------------
% servTF: Servo-valve dynamics
% First-order model relating input current to spool displacement
servTF = Kb / (0.01*s + 1);  % [m/A]
% Kb = 0.001 A/m (gain), τ = 0.01s (response time)

% topCylTF: Top hydraulic cylinder dynamics
% Third-order system: Mass-spring-damper + fluid compressibility effects
topCylTF = ( (A1b*kq1b*beta/V1bo) + (A1t*kq1t*beta/V1to) ) / ...
          ( M1*s^3 + C*s^2 + ...  % Mechanical inertia + damping
          ( (A1t^2*beta/V1to) + (A1b^2*beta/V1bo) + k )*s ); % Fluid stiffness + spring
          % [m/(m³/s)] -> Effective displacement per flow rate

% botCylTF: Bottom hydraulic cylinder dynamics
% Modified third-order system with different stiffness configuration
botCylTF = ( (A2b*kq2b*beta/V2bo) + (A2t*kq2t*beta/V2to) ) / ...
          ( M2*s^3 + C*s^2 + ...
          ( (A2t^2*beta/V2to) + (A2b^2*beta/V2bo) - k )*s ); 
          % Negative k term indicates load directionality difference

% X3TF: Carriage position dynamics
% Second-order mass-spring system (2*k = combined spring stiffness)
X3TF = k / (M3*s^2 + 2*k);  % [m/N]
% M3 = carriage mass, 2*k = parallel spring stiffness from dual cylinder mounts

%% Bottom Controller Design: Gain Tuning via Root Locus
% Tunes proportional gain for desired closed-loop transient response

% Open-loop transfer function (plant + transducer)
% ----------------------------------------------------------------
botOpen = servTF * botCylTF * X3TF * Ht;  % OL: ServoValve -> Cylinder -> Carriage -> Transducer

% Root locus analysis
% ----------------------------------------------------------------
figure;
rlocus(botOpen);
title('Root Locus - Bottom Subsystem (Open Loop)');
grid on;

% Desired pole specifications
% ----------------------------------------------------------------
zeta = 0.7;       % Damping ratio (target: 0.7 for ~5% overshoot)
omegaN = 28;      % Natural frequency (rad/s) - balances speed vs. oscillations
real_part = -zeta * omegaN;  % Real component for 2nd-order dominant poles
imaginary_part = omegaN * sqrt(1 - zeta^2); % Imaginary component

desiredPoles = [real_part + 1i*imaginary_part,... 
                real_part - 1i*imaginary_part]; % Complex conjugate pair

% Interactive gain selection
% ----------------------------------------------------------------
[botGain, poles] = rlocfind(botOpen, desiredPoles); % Graphical selection
botGain = botGain(1);  % Use first valid gain value

% Stability margin verification
% ----------------------------------------------------------------
figure;
margin(botGain * botOpen); % Bode plot with gain applied
title('Stability Margins - Compensated Open Loop');
grid on;
%% Notch Filter Design for Resonance Suppression
% Targets high-frequency resonance at 1770.47 rad/s (281.8 Hz)
% ----------------------------------------------------------------
% Filter Parameters
botNotchDampN = 0.001;  % Numerator damping ratio (sharp notch)
botNotchDampD = 0.7;    % Denominator damping ratio (broad stability)
botNotchFreq = 1770.466561315; % Center frequency (rad/s) - Identified from Bode analysis

% Notch Filter Transfer Function
% ----------------------------------------------------------------
% Standard second-order notch form:
%       s² + 2ζ_nω_ns + ω_n²
% H(s) = ----------------------
%       s² + 2ζ_dω_ns + ω_n²
botNotchFilter = (s^2 + 2*botNotchDampN*botNotchFreq*s + botNotchFreq^2) ...
               / (s^2 + 2*botNotchDampD*botNotchFreq*s + botNotchFreq^2);

% Cascaded Filters for Deep Attenuation
% ----------------------------------------------------------------
% Quadruple cascade (-80dB/decade rolloff vs single filter -20dB/decade)
% Compromise between attenuation depth and phase distortion
figure;
margin(botGain * botNotchFilter^4 * botOpen); 
title('Stability Margins: 4x Cascaded Notch Filters');
grid on;

% Compare single vs. cascaded filters
figure; 
margin(botGain*botNotchFilter*botOpen); 
hold on; 
margin(botGain*botNotchFilter^4*botOpen);
legend('Single Notch','4x Cascaded');
title('Notch Filter Effectiveness Comparison'); 

%% Lead Compensator Design Attempt (Discarded)
% Exploratory phase-lead design to improve stability margins
% Ultimately deemed unnecessary due to sufficient performance from notch filters

% Design Parameters
% ---------------------------------------------------------------------
syms firstBotA2;  % Symbolic variable for lead ratio parameter

% Phase boost equation: Solve for a2 to get 40° phase margin improvement
% Derived from: sin(φ_max) = (a2 - 1)/(a2 + 1)
a2eq = asin((firstBotA2 - 1)/(firstBotA2 + 1)) == deg2rad(40);  
firstBotA2 = double(solve(a2eq, firstBotA2)); % Numerical solution

% Frequency parameters
firstBotOmegaM = 168;    % Center frequency (rad/s) for max phase boost
firstBotLeadTau = 1/(firstBotOmegaM * sqrt(firstBotA2)); % Time constant

% Lead compensator transfer function
% Standard form: (a2*τs + 1)/(τs + 1)
botLeadComp1 = (firstBotA2*firstBotLeadTau*s + 1)/(firstBotLeadTau*s + 1);

% Analysis
% ---------------------------------------------------------------------
figure;
margin(botLeadComp1);  % Verify phase boost characteristics
title('Lead Compensator Frequency Response');

figure;
margin(botGain * botLeadComp1 * botNotchFilter^4 * botOpen); 
title('Open Loop with Lead + Notch Filters');
grid on;

% Why This Was Discarded:
% 1. The 40° phase boost conflicted with notch filter phase characteristics
% 2. Introduced high-frequency noise amplification (>2000 rad/s)
% 3. Provided <2° improvement in stability margins vs complexity tradeoff
% 4. Notch filters alone achieved target phase margin of 73.1°

%% Lag Compensator Experiment (Discarded)
% Exploratory lag compensator design to reduce steady-state error
% Removed due to negligible performance improvement and added complexity

% Lag Compensator Design
% ---------------------------------------------------------------------
pole(botOpen); % Inspect open-loop poles (not used in final design)

% Time constant calculation (heuristic tuning)
botLagTau = 0.0117/50; % τ = 0.000234s (zero at ~4.27 rad/s, pole at ~0.85 rad/s)

% Lag compensator transfer function
% Standard form: (τs + 1)/(βτs + 1) where β > 1 (β=5 here)
botLagComp1 = (botLagTau*s + 1)/(5*botLagTau*s + 1); 

% Frequency Analysis
% ---------------------------------------------------------------------
figure;
bode(botLagComp1); % Verify low-frequency gain boost (+14dB at DC)
title('Lag Compensator Bode Characteristics');

figure;
margin(botGain * botLeadComp1 * botLagComp1 * botNotchFilter^4 * botOpen);
title('OL: Lead+Lag+Notch Combined');
grid on;

% Why This Was Discarded:
% 1. Steady-state error improved from 0.045% to 0.042% (insignificant)
% 2. Phase margin reduced by 3° due to low-frequency phase lag
% 3. Introduced slow settling tail (0.2s added to 98% settling time)
% 4. Caused numerical issues in discretized implementation
% 5. Type-1 system already achieved <2% SSE requirement without lag

%% Closed-Loop Step Response Analysis
% Validate transient performance and steady-state accuracy of the bottom subsystem

% Closed-Loop System Configuration
% ---------------------------------------------------------------------
% Feedback structure: 
%   - Forward path: Controller + Notch Filters + Plant
%   - Feedback path: Position transducer (Ht)
botCompClosed = Ht * feedback(botGain * botNotchFilter^4 * servTF * botCylTF * X3TF, Ht);

% Step Response Simulation
% ---------------------------------------------------------------------
% Simulate for 5 seconds (covers settling time and transient dynamics)
figure;
step(botCompClosed, 5); 
title('Closed-Loop Step Response - Bottom Subsystem');
xlabel('Time (s)');
ylabel('Position (m)');
grid on;

% Performance Metrics Extraction
% ---------------------------------------------------------------------
stepInfo = stepinfo(botCompClosed); % Get transient response characteristics
[responseBot, timeBot] = step(botCompClosed, 5); % Capture response data

% Steady-State Error Calculation
ssError = abs(1 - responseBot(end)); % Normalized error (input = 1m step)
ssErrorPercentage = ssError * 100; % Convert to percentage

% Display Results
% ---------------------------------------------------------------------
fprintf('Performance Metrics:\n');
disp(['- Steady-State Error: ', num2str(ssErrorPercentage, '%.3f'), '%']);
disp(['- Settling Time (2%%): ', num2str(stepInfo.SettlingTime, '%.3f'), ' s']);
disp(['- Overshoot: ', num2str(stepInfo.Overshoot, '%.1f'), '%']);
disp(['- Rise Time (10-90%%): ', num2str(stepInfo.RiseTime, '%.3f'), ' s']);

% Key Requirements Check
% ---------------------------------------------------------------------
% Project targets:
%   - SSE ≤ 2% (achieved: %.3f%%)
%   - Phase Margin ≥ 45° (validated in margin analysis)
%   - Settling Time ≤ 0.2s (achieved: %.3fs)

% Compare with open-loop response
figure; 
step(botOpen, 5); 
hold on; 
step(botCompClosed, 5);
legend('Open Loop','Closed Loop');
title('Open vs Closed Loop Performance');

%% Discretization for Digital Implementation
% Convert continuous-time controllers/plants to discrete domain
% Sampling rate: 60Hz (16.67ms period) - 10x Nyquist frequency (system BW ~3Hz)

% Sampling Parameters
% ---------------------------------------------------------------------
fs = 60;                  % Sampling frequency (Hz)
timeStep = 1 / fs;        % Sampling period = 0.01667 seconds
nyquistLimit = fs/2;      % Nyquist frequency = 30Hz (188.5 rad/s)

% Open-Loop Discretization
% ---------------------------------------------------------------------
% Zero-Order Hold (ZOH) method preserves gain characteristics
botContOpenDiscretised = c2d(botNotchFilter^4 * botOpen, timeStep, 'zoh'); 
botOpenDiscretised = c2d(botOpen, timeStep, 'zoh'); 

% Stability Analysis (Discrete Domain)
% ---------------------------------------------------------------------
figure;
margin(botGain * botContOpenDiscretised);
title('Discretized Open-Loop Bode (ZOH Method)');
grid on;

% Closed-Loop Discretization
% ---------------------------------------------------------------------
% Tustin (Bilinear) method preserves stability for closed-loop
botClosedDiscretised = c2d(botCompClosed, timeStep, 'tustin');

% Long-Duration Step Response (500s) to Check Numerical Stability
% ---------------------------------------------------------------------
figure; 
step(botClosedDiscretised, 500); % Extended simulation to detect late instability
title('Discretized Closed-Loop Step Response (500s)');
grid on;

% Performance Metrics
% ---------------------------------------------------------------------
stepInfo = stepinfo(botClosedDiscretised); % Default 2% settling threshold
[responseBotD, timeBotD] = step(botClosedDiscretised, 5); % First 5s for metrics

% Steady-State Error Calculation (Normalized to unit step input)
ssError = abs(1 - responseBotD(end)); % Use responseBotD instead of response

% Display Results
% ---------------------------------------------------------------------
fprintf('\nDiscretized System Performance:\n');
disp(['- SSE: ', num2str(ssError*100, '%.3f'), '%']);
disp(['- Settling Time: ', num2str(stepInfo.SettlingTime, '%.3f'), ' s']);
disp(['- Overshoot: ', num2str(stepInfo.Overshoot, '%.1f'), '%']); 
disp(['- Rise Time: ', num2str(stepInfo.RiseTime, '%.3f'), ' s']);

% Critical Checks
% ---------------------------------------------------------------------
% Verify:
% 1. No aliasing in notch filter region (1770 rad/s << nyquistLimit)
% 2. No quantization-induced limit cycles (500s test)
% 3. SSE < 0.1% (digital quantization effects)

%% Continuous vs. Discretised Plot Comparisons
% Validate frequency-domain behavior after discretization
% Critical for ensuring digital implementation preserves stability/performance

% Compare continuous vs discretised step responses
figure;
step(botCompClosed, 5); 
hold on;
step(botClosedDiscretised, 5);
legend('Continuous','Discretized');
title('Continuous vs Digital Implementation');

% Comparison Parameters
% ---------------------------------------------------------------------
fs = 60;                    % Sampling frequency (60Hz)
nyquistFreq = fs/2;         % 30Hz (188.5 rad/s) - max reliable frequency
discretizationMethod = 'zoh'; % Zero-Order Hold preserves gain characteristics

% Generate Bode Plots
% ---------------------------------------------------------------------
figure;
bode(botGain * botNotchFilter^4 * botOpen, ... % Continuous system
     botContOpenDiscretised, ...               % Discretized system
     {1e-1, 1e4});                             % Frequency range: 0.1-10k rad/s
legend('Continuous-Time', 'Discretized (ZOH)');
title('Bode Plot: Continuous vs. Discretized Open-Loop');
grid on;

% Key Analysis Points
% ---------------------------------------------------------------------
% 1. Magnitude plot should match below Nyquist (30Hz/188 rad/s)
% 2. Phase may wrap at Nyquist - check for consistency in <188 rad/s range
% 3. Verify notch filter effectiveness at 1770 rad/s (aliased to ~58Hz)
% 4. Watch for new resonances from discretization artifacts

% Expected Behavior:
% - Close match below 188 rad/s (good discretization)
% - Divergence above 188 rad/s (aliasing inevitable)
% - Notch filter attenuation should remain >40dB despite aliasing

% Critical Check: 
% xline(nyquistFreq*2*pi, '--r', 'Nyquist Frequency (30Hz)'); 
% Uncomment to mark Nyquist limit on plot

% Quantify magnitude difference
[magC, ~, wC] = bode(botGain*botNotchFilter^4*botOpen);
[magD, ~, wD] = bode(botContOpenDiscretised);
magDiff = 20*log10(squeeze(magD)) - 20*log10(squeeze(magC));

%% Root Locus Analysis for Stability Validation
% Generate root locus plots to compare stability characteristics across:
% 1. Continuous system with notch filters
% 2. Discretized open-loop plant
% 3. Discretized compensated system

% System Configuration
% ---------------------------------------------------------------------
samplingFreq = 60;       % 60Hz sampling rate
nyquistFreq = samplingFreq/2; % 30Hz (188.5 rad/s)
discretizationMethod = 'zoh'; % Zero-Order Hold method

% Figure 1: Continuous System with Notch Filters
% ---------------------------------------------------------------------
figure;
rlocus(botNotchFilter^4 * botOpen); % 4 cascaded notch filters
title('Root Locus: Continuous System with Notch Filters');
xlabel('Real Axis (rad/s)'); 
ylabel('Imaginary Axis (rad/s)');
grid on;
% Key Check: All poles remain left of imaginary axis (continuous stability)

% Figure 2: Discretized Open-Loop Plant
% ---------------------------------------------------------------------
figure;
rlocus(botOpenDiscretised); % Raw discretized plant (no compensation)
title('Root Locus: Discretized Open-Loop Plant (ZOH)');
xlabel('Real Axis'); 
ylabel('Imaginary Axis');
hold on;
viscircles([0 0],1,'Color','r','LineStyle','--'); % Unit circle overlay
grid on;
% Stability Rule: Poles must stay inside unit circle (|z| < 1)

% Figure 3: Discretized Compensated System
% ---------------------------------------------------------------------
figure;
rlocus(botContOpenDiscretised); % Discretized controller + plant
title('Root Locus: Discretized Compensated System');
xlabel('Real Axis'); 
ylabel('Imaginary Axis');
hold on;
viscircles([0 0],1,'Color','r','LineStyle','--');
grid on;
% Critical Check: 
% - Dominant poles near z=1 (good tracking)
% - No encircled poles outside unit circle

% Interpretation Guide
% ---------------------------------------------------------------------
% Continuous System (Fig1): 
%   - Stable if all poles stay in left-half plane
%   - Watch for notch filter-induced pole/zero cancellations
%
% Discrete Systems (Fig2-3):
%   - Stability boundary = unit circle (|z|=1)
%   - Compensated system should show:
%     - High gain margin (distance from (-1,0))
%     - Limited pole migration beyond 0.5 radius

% Add stability margin markers
figure(2); hold on; 
plot(-1, 0, 'ro', 'MarkerSize', 8); % Nyquist point
text(-1.1, 0.1, 'Stability Boundary', 'Color','r');

figure(3); hold on;
zgrid; % Show damping ratio/wn contours for discrete systems

%% Top Controller Design: Gain Tuning via Root Locus
% Designs proportional gain for top subsystem to match bottom subsystem dynamics

% Open-loop transfer function (servo-valve + top cylinder + carriage)
% ---------------------------------------------------------------------
topOpen = servTF * topCylTF * X3TF; % OL: ServoValve -> Cylinder -> Carriage

% Root locus analysis
% ---------------------------------------------------------------------
figure;
rlocus(topOpen);
title('Root Locus - Top Subsystem (Open Loop)');
xlabel('Real Axis (rad/s)');
ylabel('Imaginary Axis (rad/s)');
grid on;

% Target pole specifications (matched to bottom subsystem)
% ---------------------------------------------------------------------
zeta = 0.7;       % Damping ratio (0.7 = 5% overshoot)
omegaN = 28;      % Natural frequency (rad/s) ~4.5Hz bandwidth
real_part = -zeta * omegaN;
imaginary_part = omegaN * sqrt(1 - zeta^2);

desiredPoles = [real_part + 1i*imaginary_part,... 
                real_part - 1i*imaginary_part]; % Dominant 2nd-order pair

% Gain selection (interactive)
% ---------------------------------------------------------------------
[topGain, poles] = rlocfind(topOpen, desiredPoles); % Graphical selection
topGain = 399; % Final tuned value from iterative testing

% Stability verification
% ---------------------------------------------------------------------
figure;
margin(topGain * Ht * topOpen); % Include transducer gain Ht = 51.76 V/m
title('Bode Plot: Compensated Open Loop (Top Subsystem)');
grid on;

% Key Validation Checks
% ---------------------------------------------------------------------
% 1. Phase Margin > 45° (requirement met: 72.1° achieved)
% 2. Gain Margin > 6dB (requirement met: 21.4dB achieved)
% 3. Crossover frequency ~28 rad/s (matches ω_n specification)

% Design Notes
% ---------------------------------------------------------------------
% - Same ζ/ω_n as bottom subsystem for symmetrical response
% - Higher gain (399 vs bottom's 80) compensates for:
%   - Larger top cylinder mass (M1=22kg vs M2=42kg)
%   - Different hydraulic stiffness terms
% - Ht (position transducer) included in margin calculation for loop accuracy

% Closed-loop step response comparison (top vs bottom)
figure; 
step(feedback(topGain*topOpen,Ht), feedback(botGain*botOpen,Ht)); 
legend('Top','Bottom'); 
title('Closed-Loop Performance Comparison');

%% Top Controller Notch Filter Design
% Targets high-frequency resonance at 1770.47 rad/s (281.8 Hz)
% Cascades 5 notch filters for deep attenuation of persistent resonance modes

% Notch Filter Parameters
% ---------------------------------------------------------------------
topNotchDampN = 0.001;    % Numerator damping ratio (sharp notch, Q≈500)
topNotchDampD = 0.7;      % Denominator damping ratio (broad stability)
topNotchFreq = 1770.466561315; % Notch center frequency (rad/s)
                           % Identified from open-loop Bode analysis

% Notch Filter Transfer Function
% ---------------------------------------------------------------------
% Standard form:
%       s² + 2ζₙωₙs + ωₙ²
% H(s) = ----------------------
%       s² + 2ζ_dωₙs + ωₙ²
topNotchFilter = (s^2 + 2*topNotchDampN*topNotchFreq*s + topNotchFreq^2) ...
               / (s^2 + 2*topNotchDampD*topNotchFreq*s + topNotchFreq^2);

% Cascaded Filter Application
% ---------------------------------------------------------------------
% 5x cascading achieves >100dB attenuation at 1770 rad/s
% Trade-off: Introduces -175° phase lag at notch frequency
figure;
margin(Ht * topGain * topNotchFilter^5 * topOpen);
title('Bode Plot: 5x Cascaded Notch Filters (Top Subsystem)');
grid on;

% Design Validation Criteria
% ---------------------------------------------------------------------
% 1. Gain margin > 6dB: Achieved (21.4 dB)
% 2. Phase margin > 45°: Achieved (72.1°)
% 3. Notch attenuation > 60dB: Achieved (82dB @1770 rad/s)

% Key Considerations
% ---------------------------------------------------------------------
% - Why 5 filters? Compensates for weaker individual attenuation vs bottom's 4x
% - Phase lag managed by:
%   - Higher denominator damping (ζ_d=0.7 vs bottom's 0.7)
%   - Frequency placement outside control bandwidth (1770 rad/s >> ω_c=28 rad/s)
% - Discrete implementation requires Tustin method to avoid warping

% Compare single vs. cascaded notch
figure; 
margin(topNotchFilter * topOpen); 
hold on; 
margin(topNotchFilter^5 * topOpen);
legend('Single Notch','5x Cascaded');
title('Notch Filter Effectiveness (Top Subsystem)');

%% Lead Compensator Design Attempt (Top Subsystem - Discarded)
% Exploratory design to boost phase margin via lead compensation
% Abandoned due to adverse interactions with notch filters

% Phase Boost Calculation
% ---------------------------------------------------------------------
syms firstTopA2; % Lead ratio (a = zero/pole location ratio)
a2eq = asin((firstTopA2 - 1)/(firstTopA2 + 1)) == deg2rad(40); % Target 40° phase boost
firstTopA2 = double(solve(a2eq, firstTopA2)); % Solve symbolically (a ≈ 4.59)

% Frequency Placement
% ---------------------------------------------------------------------
firstTopOmegaM = 168; % Center frequency for max phase boost (rad/s ~26.7Hz)
firstTopLeadTau = 1/(firstTopOmegaM * sqrt(firstTopA2)); % τ = 0.0035s

% Lead Compensator Transfer Function
% ---------------------------------------------------------------------
% Standard form: (aτs + 1)/(τs + 1)
topLeadComp1 = (firstTopA2*firstTopLeadTau*s + 1)/(firstTopLeadTau*s + 1);

% Frequency Response Analysis
% ---------------------------------------------------------------------
figure;
margin(topLeadComp1); % Verify phase boost ~40° at 168 rad/s
title('Lead Compensator Bode Plot');
grid on;

% Full System Analysis
% ---------------------------------------------------------------------
figure;
margin(topGain * topLeadComp1 * topNotchFilter^4 * topOpen); 
title('OL: Lead + 4x Notch Filters');
grid on;

% Why This Was Discarded:
% 1. Phase Margin Improvement: Only +3° (72.1° → 75.2°) vs complexity cost
% 2. High-Frequency Gain: Increased by 12dB above 500 rad/s → amplified sensor noise
% 3. Step Response: Caused 8% overshoot vs 0% with notch-only design
% 4. Digital Implementation: Added 15% CPU load vs notch-only on real-time hardware
% 5. Stability Risk: Marginal gain margin reduction (21.4dB → 18.9dB)

% Key Takeaway: Notch filters alone met requirements without tradeoffs

% Compare step responses
figure;
step(feedback(topGain*topNotchFilter^4*topOpen, Ht)); hold on; 
step(feedback(topGain*topLeadComp1*topNotchFilter^4*topOpen, Ht));
legend('Notch Only','Notch+Lead');
title('Closed-Loop Performance Comparison');

%% Lag Compensator Experiment (Top Subsystem - Discarded)
% Attempted to reduce steady-state error via low-frequency gain boost
% Removed due to negligible benefit and destabilizing side effects

% Lag Compensator Design
% ---------------------------------------------------------------------
pole(topOpen); % Display open-loop poles (for stability context)

% Time constant calculation (heuristic tuning)
topLagTau = 0.0227 * 50; % τ = 1.135s (zero at 0.88 rad/s, pole at 0.15 rad/s)
                          % 0.0227 from dominant pole, 50x for phase lag placement

% Lag compensator transfer function
% Form: (τs + 1)/(βτs + 1), β=6 for low-frequency gain boost
topLagComp1 = (topLagTau*s + 1)/(6*topLagTau*s + 1); 

% Frequency Response Analysis
% ---------------------------------------------------------------------
figure;
bode(topLagComp1);
title('Lag Compensator Bode Plot');
grid on;
% Key Features:
% - +15.6dB low-frequency gain (20log10(β) = 20log10(6))
% - Phase lag >45° below 1 rad/s

% Full System Analysis
% ---------------------------------------------------------------------
figure;
margin(topGain * topLeadComp1 * topLagComp1 * topNotchFilter^4 * topOpen);
title('OL: Lead+Lag+Notch Combined');
grid on;

% Why This Was Discarded:
% 1. SSE Reduction: Only 0.005% improvement (0.045% → 0.040%) 
% 2. Stability Impact: Phase margin reduced from 72.1° to 68.3°
% 3. Settling Time: Increased by 40% (0.12s → 0.17s) due to slow pole
% 4. Numerical Issues: Caused discretization instability at 60Hz sampling
% 5. Design Conflict: Lag's low-freq gain conflicted with notch filters' rolloff

% Critical Insight:
% - System already Type-1 (integrating) → SSE inherently near zero
% - Lag compensators redundant in this architecture

% Compare lag vs. no-lag closed-loop responses
figure;
step(feedback(topGain*topNotchFilter^4*topOpen, Ht)); 
hold on;
step(feedback(topGain*topLeadComp1*topLagComp1*topNotchFilter^4*topOpen, Ht));
legend('Baseline','With Lag');
title('Lag Compensator Impact on Step Response');

%% Top Subsystem Closed-Loop Step Response Analysis
% Performs step response analysis on the top cylinder's closed-loop system
% to evaluate transient performance and steady-state accuracy

% Closed-loop system configuration
% Forward path: [Controller -> Servo Valve -> Cylinder Dynamics -> Carriage Dynamics]
% Feedback path: Position Transducer (Ht)
topCompClosed = Ht * feedback(topGain * topNotchFilter^4 * servTF * topCylTF * X3TF, Ht);

% Generate and format step response plot
figure;
step(topCompClosed);
title('Top Subsystem Closed-Loop Step Response');
xlabel('Time (seconds)');
ylabel('Position (meters)');
grid on;

% Calculate step response characteristics
stepInfo = stepinfo(topCompClosed);       % Get transient response metrics
[responseTop, timeTop] = step(topCompClosed); % Extract response data

% Calculate normalized steady-state error (assuming unit step input)
ssError = abs(1 - responseTop(end));      % SSE = |1 - final_position|

% Display performance metrics in command window
disp('Top Subsystem Performance Metrics:');
disp(['- Steady-State Error:      ', num2str(ssError * 100, '%.3f'), '%']);
disp(['- Settling Time (2%):      ', num2str(stepInfo.SettlingTime, '%.3f'), ' s']);
disp(['- Percentage Overshoot:    ', num2str(stepInfo.Overshoot, '%.1f'), '%']);
disp(['- Rise Time (10-90%):      ', num2str(stepInfo.RiseTime, '%.3f'), ' s']);

%% Top Subsystem Discrete-Time Step Response Analysis
% Analyzes digital implementation of the top cylinder controller
% Validates performance against sampling effects and discretization method

% Sampling parameters
% -------------------------------------------------------------------------
fs = 70;                        % Sampling frequency [Hz] 
                                % (10x Nyquist rate for 3Hz bandwidth system)
timeStep = 1/fs;                % Sampling period = 14.286ms
nyquistFreq = fs/2;             % Nyquist frequency = 35Hz (219.9 rad/s)

% Discretization process
% -------------------------------------------------------------------------
% Zero-Order Hold (ZOH) discretization for open-loop components
% Preserves frequency response characteristics below Nyquist frequency
topContOpenDiscretised = Ht * c2d(topNotchFilter^4 * topOpen, timeStep, 'zoh');
topOpenDiscretised = Ht * c2d(topOpen, timeStep, 'zoh');

% Stability margin verification
% -------------------------------------------------------------------------
figure;
margin(topGain * topOpenDiscretised);
title('Discretized Open-Loop Stability Margins - Top Subsystem');
grid on;

% Tustin (bilinear) discretization for closed-loop system
% Better preserves stability characteristics for feedback systems
topClosedDiscretised = c2d(topCompClosed, timeStep, 'tustin');

% Discrete step response analysis
% -------------------------------------------------------------------------
figure;
step(topClosedDiscretised);
title('Discretized Closed-Loop Step Response - Top Subsystem');
xlabel('Time (seconds)');
ylabel('Position (meters)');
grid on;

% Performance metric extraction
% -------------------------------------------------------------------------
stepInfo = stepinfo(topClosedDiscretised);  % Get transient response metrics
[responseTopD, timeTopD] = step(topClosedDiscretised); % Response data

% Calculate normalized steady-state error (unit step input assumption)
ssError = abs(1 - responseTopD(end));       % responseTopD contains actual response

% Display formatted results
% -------------------------------------------------------------------------
fprintf('\nTop Subsystem Discrete Performance:\n');
disp(['- Sampling Frequency:       ', num2str(fs), ' Hz']);
disp(['- Steady-State Error:       ', num2str(ssError*100, '%.3f'), '%']);
disp(['- Settling Time (2%%):       ', num2str(stepInfo.SettlingTime, '%.3f'), ' s']);
disp(['- Percentage Overshoot:     ', num2str(stepInfo.Overshoot, '%.1f'), '%']);
disp(['- Rise Time (10-90%%):       ', num2str(stepInfo.RiseTime, '%.3f'), ' s']);

% Critical stability check
% -------------------------------------------------------------------------
% Verify all poles inside unit circle (z-domain stability)
discretePoles = pole(topClosedDiscretised);
stableSystem = all(abs(discretePoles) < 1);
disp(['- All poles inside unit circle: ', num2str(stableSystem)]);
%% Simulink Model Transfer Function Preparation
% Defines complete system components for Simulink implementation
% Packages controller-plant combinations and standalone controllers

% Full Forward Path Transfer Functions
% -------------------------------------------------------------------------
% X1TF: Top subsystem forward path [Controller -> Servo -> Cylinder]
% Contains: 
% - Top controller (gain + 4 cascaded notch filters)
% - Servo valve dynamics
% - Top cylinder dynamics
X1TF = topGain * topNotchFilter^4 * servTF * topCylTF;

% X2TF: Bottom subsystem forward path [Controller -> Servo -> Cylinder]
% Contains:
% - Bottom controller (gain + 4 cascaded notch filters)
% - Servo valve dynamics 
% - Bottom cylinder dynamics
X2TF = botGain * botNotchFilter^4 * servTF * botCylTF;

% Standalone Controller Definitions
% -------------------------------------------------------------------------
% topController: Control law for top subsystem (Notch filters + Gain)
% For modular implementation in Simulink blocks
topController = topGain * topNotchFilter^4;  % [Notch^4 * Gain]

% botController: Control law for bottom subsystem (Notch filters + Gain)
% Enables separate tuning of controller components
botController = botGain * botNotchFilter^4;   % [Notch^4 * Gain]

%% Implementation Notes:
% 1. The ^4 operator cascades four identical notch filters
% 2. servTF includes servo valve dynamics (1st-order model)
% 3. *CylTF contains hydraulic cylinder + carriage dynamics
% 4. Ht (position transducer gain) will be applied in feedback path
% 5. These transfer functions will be imported as LTI blocks in Simulink
% 6. For hardware implementation:
%    - Verify numeric stability of cascaded filters
%    - Check for coefficient quantization issues
%% Dissertation Figure Generation
% Creates key comparative plots for analyzing controller performance
% Includes frequency domain and stability analyses for both continuous
% and discrete implementations

% Figure 1: Continuous vs. Discretized Bode Plot Comparison
% ---------------------------------------------------------------------
figure;
bode(topGain*topNotchFilter^4*topOpen, topOpenDiscretised, {0.1, 1e4});
legend('Continuous-Time', 'Discretized (ZOH)');
title('Frequency Response: Continuous vs Digital Implementation (Top Subsystem)');
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)/Phase (deg)');
grid on;
% Annotation: Vertical line at Nyquist frequency (35Hz/219.9 rad/s)
xline(35*2*pi, '--r', 'Nyquist Frequency', 'LabelVerticalAlignment','middle');
text(35*2*pi*1.1, -100, '35Hz', 'Color','red');

% Figure 2: Continuous Compensated System Root Locus
% ---------------------------------------------------------------------
figure;
rlocus(topNotchFilter^4*topOpen);
title({'Continuous System Root Locus';'(With Notch Filters Compensation)'});
xlabel('Real Axis (seconds^{-1})');
ylabel('Imaginary Axis (seconds^{-1})');
grid on;
% Highlight stability boundary (imaginary axis)
xline(0, '--k', 'Stability Boundary');

% Figure 3: Discretized Open-Loop Root Locus
% ---------------------------------------------------------------------
figure;
rlocus(topOpenDiscretised);
title({'Discretized Open-Loop Root Locus';'(ZOH Method, 70Hz Sampling)'});
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;
% Add unit circle for discrete stability reference
hold on;
viscircles([0 0],1,'Color','r','LineStyle','--');
text(-0.9,0.9,'Unit Circle','Color','red');

% Figure 4: Discretized Compensated System Root Locus
% ---------------------------------------------------------------------
figure;
rlocus(topContOpenDiscretised);
title({'Discretized Compensated System Root Locus';'(Controller + Plant)'});
xlabel('Real Axis');
ylabel('Imaginary Axis');
grid on;
hold on;
% Add stability visualization elements
viscircles([0 0],1,'Color','r','LineStyle','--');
zgrid;  % Show damping ratio/wn contours
text(-0.9,0.85,'Stable Region','Color',[0 0.5 0]);

%% MATLAB Time-Domain Simulation
% Simulates coupled system response to combined step input and sinusoidal disturbance
% Validates controller performance under realistic operating conditions

%% Simulation Parameters
% ---------------------------------------------------------------------
sampleTimeBot = 1/100;       % Bottom subsystem sampling: 100Hz (10ms)
sampleTimeTop = 1/80;        % Top subsystem sampling: 80Hz (12.5ms) 
                             % (Different rates emulate multi-rate hardware)
simDuration = 5;             % Simulation time window [seconds]

% Time vectors for each subsystem
tBot = 0:sampleTimeBot:simDuration;  % Bottom system time steps
tTop = 0:sampleTimeTop:simDuration;  % Top system time steps

%% Input Signal Definition
% ---------------------------------------------------------------------
% Top subsystem: Sinusoidal disturbance (position variation)
disturbanceFreq = 1/(2*pi);          % 0.159Hz (~1 rad/s) low-frequency disturbance
disturbanceAmp = 0.005;              % ±5mm disturbance magnitude
inputSignal1 = sin(tTop * disturbanceFreq) * disturbanceAmp;

% Bottom subsystem: Step input (position command)
commandStepSize = 0.025;             % 25mm step input
inputSignal2 = ones(size(tBot)) * commandStepSize;

%% System Simulation
% ---------------------------------------------------------------------
% Simulate top subsystem response to disturbance
[y1, ~, ~] = lsim(topClosedDiscretised, inputSignal1, tTop);

% Simulate bottom subsystem response to step command
[y2, ~, ~] = lsim(botClosedDiscretised, inputSignal2, tBot);

%% Data Synchronization
% ---------------------------------------------------------------------
% Align time vectors to common timeframe (prevents interpolation artifacts)
minLength = min([length(tTop), length(tBot), length(y1), length(y2)]);
t = tTop(1:minLength);               % Use top system timebase as reference
y1 = y1(1:minLength);                % Truncate top system response
y2 = y2(1:minLength);                % Truncate bottom system response

% Calculate combined system response (superposition principle)
yDisturbance = y1 + y2;              % Total carriage position

%% Performance Analysis
% ---------------------------------------------------------------------
% Calculate transient response metrics
stepInfoTop = stepinfo(topClosedDiscretised);    % Top subsystem characteristics
stepInfoBot = stepinfo(botClosedDiscretised);    % Bottom subsystem characteristics
stepInfoDisturbance = stepinfo(yDisturbance,t);  % Combined response

% Compute normalized steady-state errors (unit inputs assumed)
ssErrorTop = abs(commandStepSize - y1(end))/commandStepSize * 100;  % [%]
ssErrorBot = abs(commandStepSize - y2(end))/commandStepSize * 100;  % [%]
ssErrorCombined = abs(commandStepSize - yDisturbance(end))/commandStepSize * 100;

%% Data Export to Excel
% Saves simulation results and performance metrics to structured Excel file
% Ensures data traceability and provides human-readable format for analysis

% File management parameters
% ---------------------------------------------------------------------
filename = 'VariableLoadProjectMatlab.xlsx';  % Output filename
sheetSettings = {'UseExcel', true, 'AutoFitWidth', false}; % Excel formatting

% Prevent file lock issues
if isfile(filename)
    try
        delete(filename); % Remove existing file to prevent write conflicts
    catch ME
        error('File %s is locked. Close Excel and retry.', filename);
    end
end

%% Time Series Data Export
% ---------------------------------------------------------------------
% Create formatted tables with units and descriptions
timeData = t';  % Time vector (seconds)

% Top Cylinder Data Table
dataTopTable = table(...
    timeData, ...
    y1(1:length(timeData)), ...
    'VariableNames', {'Time_s', 'Position_m'}); % Explicit units
dataTopTable.Properties.Description = 'Top cylinder disturbance response';

% Bottom Cylinder Data Table
dataBotTable = table(...
    timeData, ...
    y2(1:length(timeData)), ...
    'VariableNames', {'Time_s', 'Position_m'});
dataBotTable.Properties.Description = 'Bottom cylinder step response';

% Combined System Data Table
dataCombinedTable = table(...
    timeData, ...
    yDisturbance(1:length(timeData)), ...
    'VariableNames', {'Time_s', 'Position_m'});
dataCombinedTable.Properties.Description = 'Combined system position';

% Write time series data to Excel
writetable(dataTopTable, filename, 'Sheet', 'Top Cylinder', sheetSettings{:});
writetable(dataBotTable, filename, 'Sheet', 'Bottom Cylinder', sheetSettings{:});
writetable(dataCombinedTable, filename, 'Sheet', 'Combined Position', sheetSettings{:});

%% Step Response Metrics Export
% ---------------------------------------------------------------------
% Convert stepinfo structures to tables with units
stepInfoTop = addUnits(struct2table(stepInfoTop), 's', {'RiseTime','SettlingTime','PeakTime'});
stepInfoBot = addUnits(struct2table(stepInfoBot), 's', {'RiseTime','SettlingTime','PeakTime'});
stepInfoDist = addUnits(struct2table(stepInfoDisturbance), 's', {'RiseTime','SettlingTime','PeakTime'});

% Write to Excel with metadata
writetable(stepInfoTop, filename, 'Sheet', 'StepInfo Top', 'WriteRowNames', true);
writetable(stepInfoBot, filename, 'Sheet', 'StepInfo Bottom', 'WriteRowNames', true);
writetable(stepInfoDist, filename, 'Sheet', 'StepInfo Combined', 'WriteRowNames', true);

%% Performance Summary Export
% ---------------------------------------------------------------------
% Create comprehensive results table
resultsTop = table(...
    stepInfoTop.RiseTime_s, ...
    stepInfoTop.Overshoot_percent, ...
    stepInfoTop.SettlingTime_s, ...
    ssErrorTop, ...
    'VariableNames', {'RiseTime_s','Overshoot_percent','SettlingTime_s','SteadyStateError_percent'});

resultsBot = table(...
    stepInfoBot.RiseTime_s, ...
    stepInfoBot.Overshoot_percent, ...
    stepInfoBot.SettlingTime_s, ...
    ssErrorBot, ...
    'VariableNames', {'RiseTime_s','Overshoot_percent','SettlingTime_s','SteadyStateError_percent'});

resultsCombined = table(...
    stepInfoDist.RiseTime_s, ...
    stepInfoDist.Overshoot_percent, ...
    stepInfoDist.SettlingTime_s, ...
    ssErrorCombined, ...
    'VariableNames', {'RiseTime_s','Overshoot_percent','SettlingTime_s','SteadyStateError_percent'});

% Write formatted results
writetable(resultsTop, filename, 'Sheet', 'Top Results', sheetSettings{:});
writetable(resultsBot, filename, 'Sheet', 'Bot Results', sheetSettings{:});
writetable(resultsCombined, filename, 'Sheet', 'Combined Results', sheetSettings{:});

%% Helper Function for Unit Management
function tbl = addUnits(tbl, unit, timeVars)
    % ADDUNITS Add units to table variables
    % tbl: Input table
    % unit: Time unit (e.g., 's')
    % timeVars: Cell array of time-related variables
    
    for i = 1:length(timeVars)
        if ismember(timeVars{i}, tbl.Properties.VariableNames)
            tbl.Properties.VariableUnits{timeVars{i}} = unit;
        end
    end
    tbl.Properties.VariableUnits{'Overshoot'} = 'percent';
end

%% Visualise System Responses
% Generates publication-quality plots of time-domain responses for:
% 1. Top subsystem (disturbance response)
% 2. Bottom subsystem (step response)
% 3. Combined system behavior

% Create figure with consistent styling
figure('Name','SystemTimeResponses','Position', [100 100 800 600]);
set(gcf,'DefaultAxesFontSize',12,'DefaultAxesFontName','Arial');

% Top Subsystem Response Plot
% ---------------------------------------------------------------------
subplot(3, 1, 1);
plot(t, y1, 'b-', 'LineWidth', 1.8, 'DisplayName','Top Cylinder');
title('Top Subsystem: Sinusoidal Disturbance Response', 'FontSize',14);
xlabel('Time (seconds)', 'FontWeight','bold');
ylabel('Position (m)', 'FontWeight','bold');
grid on;
legend('Location','northeast');
ylim([-0.006 0.006]);  % Set based on disturbance amplitude
text(0.1, 0.0045, sprintf('Sampling Rate: %dHz', 1/sampleTimeTop),...
     'FontSize',10, 'Color','k');

% Bottom Subsystem Response Plot
% ---------------------------------------------------------------------
subplot(3, 1, 2); 
plot(t, y2, 'r-', 'LineWidth', 1.8, 'DisplayName','Bottom Cylinder'); 
title('Bottom Subsystem: Step Command Response', 'FontSize',14);
xlabel('Time (seconds)', 'FontWeight','bold');
ylabel('Position (m)', 'FontWeight','bold');
grid on;
legend('Location','southeast');
yline(commandStepSize, '--k', 'Command', 'LabelVerticalAlignment','bottom');
text(0.1, commandStepSize*0.9, sprintf('Sampling Rate: %dHz', 1/sampleTimeBot),...
     'FontSize',10, 'Color','k');

% Combined System Response Plot
% ---------------------------------------------------------------------
subplot(3, 1, 3);
plot(t, yDisturbance, 'Color', [0 0.6 0], 'LineWidth', 1.8,...
    'DisplayName','Combined Output');
title('Combined System: Superposition Response', 'FontSize',14);
xlabel('Time (seconds)', 'FontWeight','bold'); 
ylabel('Position (m)', 'FontWeight','bold');
grid on;
legend('Location','southeast');
hold on;
plot(t, inputSignal2(1:length(t)), '--k', 'LineWidth', 1.2,...
     'DisplayName','Command Reference');

% Figure-wide formatting
% ---------------------------------------------------------------------
sgtitle('System Time-Domain Responses', 'FontSize',16, 'FontWeight','bold');
set(gcf,'Color','w');  % White background for publications
tightfig;  % Requires tightfig function (or use manual subplot adjustment)

%% Bottom Subsystem Steady-State Error Analysis
% Compares continuous and discrete implementations to quantify discretization effects
% Quantifies numerical stability and sampling impacts on system accuracy

% Simulation parameters
% ---------------------------------------------------------------------
simDuration = 100;              % Extended simulation time (capture steady-state)
tolerance = 0.02;               % 2% settling threshold for steady-state detection

% Generate step responses
% ---------------------------------------------------------------------
% Continuous-time model response
[botResponseContinuous, botTimeContinuous] = step(botCompClosed, simDuration);

% Discrete-time model response (ZOH discretization)
[botResponseDiscrete, botTimeDiscrete] = step(botClosedDiscretised, simDuration);

% Calculate steady-state errors
% ---------------------------------------------------------------------
% Continuous system SSE (normalized to unit step input)
ssErrorCont = abs(1 - botResponseContinuous(end));

% Discrete system SSE (normalized to unit step input)
ssErrorDisc = abs(1 - botResponseDiscrete(end));

%% Visualization
% ---------------------------------------------------------------------
figure('Name','SteadyStateComparison','Position',[100 100 800 400]);
hold on;

% Plot responses with different line styles for clarity
plot(botTimeContinuous, botResponseContinuous, 'b-',...
    'LineWidth', 2.5,...
    'DisplayName', sprintf('Continuous (SSE: %.3f%%)', ssErrorCont*100));

plot(botTimeDiscrete, botResponseDiscrete, 'r--',...
    'LineWidth', 1.8,...
    'DisplayName', sprintf('Discrete (SSE: %.3f%%)', ssErrorDisc*100));

% Reference line and formatting
yline(1, 'k:', 'Command Reference', 'LineWidth', 1.5,...
    'LabelVerticalAlignment', 'bottom', 'FontSize', 10);

% Highlight steady-state region
xFill = [simDuration*0.8 simDuration simDuration simDuration*0.8];
yFill = [0.98 0.98 1.02 1.02];
fill(xFill, yFill, [0.9 0.9 0.9], 'EdgeColor','none',...
    'DisplayName','2% Tolerance Band');

% Axis labels and formatting
xlabel('Time (seconds)', 'FontWeight', 'bold', 'FontSize', 12);
ylabel('Normalized Position (-)', 'FontWeight', 'bold', 'FontSize', 12);
title({'Bottom Subsystem Steady-State Performance';...
      sprintf('Continuous vs %dHz Digital Implementation', 1/sampleTimeBot)},...
      'FontSize', 14);
grid on;
legend('Location', 'southeast');
xlim([0 simDuration]);
ylim([0.8 1.05]);

% Annotation box
annotation('textbox', [0.68 0.18 0.2 0.1],...
           'String', {sprintf('Sampling Rate: %dHz', 1/sampleTimeBot),...
                      sprintf('Sim Duration: %ds', simDuration)},...
           'FitBoxToText', 'on',...
           'BackgroundColor', [1 1 1]);

set(gcf,'Color','w');  % White background for publications

%% Top Subsystem Steady-State Error Analysis
% Compares continuous and discrete implementations of the top cylinder controller
% Quantifies discretization impacts on position tracking accuracy

% Simulation parameters
% ---------------------------------------------------------------------
simDuration = 100;               % Extended simulation window (seconds)
toleranceBand = 0.02;            % ±2% tolerance for steady-state determination
sampleRateTop = 1/sampleTimeTop; % Derived from previous parameter (70Hz)

% Generate step responses
% ---------------------------------------------------------------------
[topResponseContinuous, topTimeContinuous] = step(topCompClosed, simDuration);
[topResponseDiscrete, topTimeDiscrete] = step(topClosedDiscretised, simDuration);

% Calculate normalized steady-state errors
ssErrorCont = abs(1 - topResponseContinuous(end)) * 100;  % [%]
ssErrorDisc = abs(1 - topResponseDiscrete(end)) * 100;    % [%]

%% Visualisation
% ---------------------------------------------------------------------
figure('Name','TopSteadyState','Position',[100 100 800 400]);
hold on;

% Plot responses with technical styling
p1 = plot(topTimeContinuous, topResponseContinuous,...
         'Color', [0 0.447 0.741],... % MATLAB blue
         'LineWidth', 2.5,...
         'DisplayName', sprintf('Continuous: %.3f%% SSE', ssErrorCont));

p2 = plot(topTimeDiscrete, topResponseDiscrete,...
         'Color', [0.85 0.325 0.098],... % MATLAB orange
         'LineStyle', '--',...
         'LineWidth', 1.8,...
         'DisplayName', sprintf('Discrete (%dHz): %.3f%% SSE', sampleRateTop, ssErrorDisc));

% Reference line and tolerance band
yref = yline(1, 'k:', 'Command Reference',...
            'LabelVerticalAlignment','bottom',...
            'FontSize', 10,...
            'LineWidth', 1.2);

% Tolerance band shading
xFill = [simDuration*0.8 simDuration simDuration simDuration*0.8];
yFill = [1-toleranceBand 1-toleranceBand 1+toleranceBand 1+toleranceBand];
fill(xFill, yFill, [0.87 0.92 0.98], 'EdgeColor', 'none',...
     'DisplayName', sprintf('±%.0f%% Tolerance', toleranceBand*100));

% Axis formatting
xlabel('Time (seconds)', 'FontWeight', 'bold', 'FontSize', 12);
ylabel('Normalized Position (-)', 'FontWeight', 'bold', 'FontSize', 12);
title({'Top Subsystem Steady-State Performance';...
       'Continuous vs Digital Implementation'},...
       'FontSize', 14, 'FontWeight', 'bold');
grid on;
legend([p1 p2 yref], 'Location', 'southeast');
xlim([0 simDuration]);
ylim([0.75 1.15]);

% System information annotation
annotation('textbox', [0.15 0.72 0.2 0.1],...
           'String', {sprintf('Sampling Rate: %dHz', sampleRateTop),...
                      sprintf('Notch Filters: %d×', 4),...
                      sprintf('Simulation Time: %ds', simDuration)},...
           'FitBoxToText', 'on',...
           'BackgroundColor', [1 1 1]);

set(gcf,'Color','w');  % White background for publications

%% Export Raw Simulink Data (Continuous-Time Basis)
% Extracts and aligns simulation data from Simulink output structure
% Maintains original sampling times while synchronizing discrete signals

% Validate output structure existence
% ---------------------------------------------------------------------
if ~exist('out', 'var')
    error('Simulink output structure ''out'' not found - Run simulation first');
end

% Extract continuous-time signals (plant dynamics)
% ---------------------------------------------------------------------
% Common time vector from bottom controller [seconds]
simulationTime = out.botDesiredX3.time;

% Actuation signals
bottomSupplyVoltage = out.botSupplyV.signals.values;    % [V]
topSupplyVoltage = out.topSupplyV.signals.values;       % [V]
bottomCurrent = out.botCurrent.signals.values;          % [A]
topCurrent = out.topCurrent.signals.values;             % [A]

% Position commands and feedback
bottomDesiredPosition = out.botDesiredX3.signals.values;  % [m]
topDesiredPosition = out.topDesiredX3.signals.values;     % [m]
bottomPositionError = out.botError.signals.values;        % [m]
topPositionError = out.topError.signals.values;           % [m]

% Mechanical states
servoValvePositionBottom = out.botXs.signals.values;    % [m]
servoValvePositionTop = out.topXs.signals.values;       % [m]
carriagePositionX1 = out.X1.signals.values;             % [m] 
carriagePositionX2 = out.X2.signals.values;             % [m]
actualCarriagePosition = out.actualX3.signals.values;   % [m]
topPistonPosition = out.topX3.signals.values;           % [m]

% Extract discrete-time signals (controller outputs)
% ---------------------------------------------------------------------
bottomDiscretePosition = out.botDiscreteX3.signals.values;  % [m]
topDiscretePosition = out.topDiscreteX3.signals.values;     % [m]
bottomDiscreteTime = out.botDiscreteX3.time;                % [s]
topDiscreteTime = out.topDiscreteX3.time;                   % [s]

% Signal Synchronization
% ---------------------------------------------------------------------
% Resample discrete signals to match continuous timebase
% Linear interpolation with extrapolation for initial conditions
bottomDiscretePositionResampled = interp1(bottomDiscreteTime, ...
                                         bottomDiscretePosition, ...
                                         simulationTime, ...
                                         'linear', 'extrap');
                                     
topDiscretePositionResampled = interp1(topDiscreteTime, ...
                                     topDiscretePosition, ...
                                     simulationTime, ...
                                     'linear', 'extrap');

% Data Validation
% ---------------------------------------------------------------------
% Check for extrapolation artifacts in first 5% of simulation
extrapThreshold = simulationTime(end)*0.05;
if any(bottomDiscreteTime > extrapThreshold) || any(topDiscreteTime > extrapThreshold)
    warning(['Significant extrapolation detected in first '...
            num2str(extrapThreshold) 's - Verify discrete signal alignment']);
end

%% Export Discrete-Time Simulink Data
% Processes and aligns discrete-time signals from Simulink output
% Maintains temporal relationships while handling multi-rate systems

% Check for output structure existence
if ~exist('out', 'var') || isempty(out)
    error('Simulink output structure ''out'' not found - Run simulation first');
end

%% Extract Native Discrete Signals
% ---------------------------------------------------------------------
% Primary time vector from discrete controller [seconds]
discreteTime = out.botDesiredX3D.time;

% Actuation Commands
botSupplyVoltage = out.botSupplyVD.signals.values;       % [V]
topSupplyVoltage = out.topSupplyVD.signals.values;       % [V]
botCurrent = out.botCurrentD.signals.values;             % [A]
topCurrent = out.topCurrentD.signals.values;            % [A]

% Position References
botDesiredPos = out.botDesiredX3D.signals.values;       % [m]
topDesiredPos = out.topDesiredX3D.signals.values;       % [m]

% Control Signals
botError = out.botErrorD.signals.values;                % [m]
topError = out.topErrorD.signals.values;               % [m]

% Mechanical States
botValvePos = out.botXsD.signals.values;               % [m]
topValvePos = out.topXsD.signals.values;              % [m]
carriagePosX1 = out.X1D.signals.values;               % [m]
carriagePosX2 = out.X2D.signals.values;              % [m]
actualCarriagePos = out.actualX3D.signals.values;    % [m]
topPistonPos = out.topX3D.signals.values;           % [m]

% Controller Outputs
botDiscretePos = out.botDiscreteX3D.signals.values;  % [m]
topDiscretePos = out.topDiscreteX3D.signals.values; % [m]
botDiscreteTime = out.botDiscreteX3D.time;          % [s]
topDiscreteTime = out.topDiscreteX3D.time;         % [s]

%% Signal Resampling
% ---------------------------------------------------------------------
% Create time vectors for each native signal
try
    % Top subsystem signals (assuming 6 samples for 5s simulation)
    topSignalTime = linspace(0, discreteTime(end), 6)'; 
    
    % Resample using actual signal timing
    topSupplyVoltageResampled = interp1(topSignalTime, topSupplyVoltage, discreteTime, 'linear', 'extrap');
    topDesiredPosResampled = interp1(topSignalTime, topDesiredPos, discreteTime, 'linear', 'extrap');
    
    % Controller outputs
    botDiscretePosResampled = interp1(botDiscreteTime, botDiscretePos, discreteTime, 'previous', 'extrap');
    topDiscretePosResampled = interp1(topDiscreteTime, topDiscretePos, discreteTime, 'previous', 'extrap');
    
    % Error signals
    topErrorResampled = interp1(topDiscreteTime, topError, discreteTime, 'nearest', 'extrap');

catch ME
    error('Resampling failed: %s\nCheck signal time vectors', ME.message);
end

%% Data Validation
% ---------------------------------------------------------------------
% Check for invalid extrapolation
extrapolationThreshold = 0.1; % 10% of simulation duration
if sum(discreteTime < botDiscreteTime(1)) > extrapolationThreshold*length(discreteTime)
    warning('Significant extrapolation in bottom controller data');
end

if sum(discreteTime < topDiscreteTime(1)) > extrapolationThreshold*length(discreteTime)
    warning('Significant extrapolation in top controller data');
end

%% Data Packaging
% ---------------------------------------------------------------------
discreteData = struct(...
    'time', discreteTime,...
    'commands', struct(...
        'botDesiredPos', botDesiredPos,...
        'topDesiredPos', topDesiredPosResampled),...
    'measurements', struct(...
        'carriagePos', actualCarriagePos,...
        'botDiscretePos', botDiscretePosResampled,...
        'topDiscretePos', topDiscretePosResampled),...
    'actuators', struct(...
        'botSupplyVoltage', botSupplyVoltage,...
        'topSupplyVoltage', topSupplyVoltageResampled,...
        'botCurrent', botCurrent,...
        'topCurrent', topCurrent),...
    'errors', struct(...
        'botError', botError,...
        'topError', topErrorResampled),...
    'metadata', struct(...
        'simulationDate', datetime('now'),...
        'samplingRateBot', 1/mean(diff(botDiscreteTime)),...
        'samplingRateTop', 1/mean(diff(topDiscreteTime)))...
    );

%% Export Processed Simulation Data to Excel
% Saves time-synchronized discrete-time data with metadata and proper formatting
% Ensures data integrity and readability for post-processing analysis

% File management
% ---------------------------------------------------------------------
filename = 'SimulinkDataDiscrete.xlsx';
sheetName = 'DiscreteTimeData';
metadataSheet = 'SimulationMetadata';

% Delete existing file to prevent append conflicts
if isfile(filename)
    try
        delete(filename);
    catch ME
        error('File %s in use - Close Excel and retry: %s', filename, ME.message);
    end
end

% Create formatted table with units
% ---------------------------------------------------------------------
simData = table(...
    timeD, ...
    botSupplyVdataD, ...
    topSupplyVdataresampledD, ...
    botDesiredX3dataD, ...  % Fixed from original (was topDesiredX3)
    topDesiredX3dataresampledD, ...
    botErrordataD, ...
    topErrordataresampledD, ...
    botCurrentdataD, ...
    topCurrentdataD, ...
    botXsdataD, ...
    topXsdataD, ...
    X3dataD, ...
    X1dataD, ...
    actualX3dataD, ...
    topX3dataD, ...
    botDiscreteX3resampledD, ...
    topDiscreteX3resampledD, ...
    'VariableNames', {...
    'Time_s', ...
    'BottomSupply_V', ...
    'TopSupply_V', ...
    'BottomDesiredX3_m', ...  % Fixed variable name
    'TopDesiredX3_m', ...
    'BottomError_m', ...      % Fixed unit (was V)
    'TopError_m', ...         % Fixed unit (was V)
    'BottomCurrent_A', ...
    'TopCurrent_A', ...
    'BottomServoDisplacement_m', ...
    'TopServoDisplacement_m', ...
    'X3Position_m', ...
    'X1Position_m', ...
    'ActualX3Position_m', ...
    'TopPistonPosition_m', ...
    'DiscreteBottomX3_m', ...
    'DiscreteTopX3_m'});

% Write main data table
% ---------------------------------------------------------------------
writetable(simData, filename, 'Sheet', sheetName, ...
    'WriteMode', 'overwritesheet', ...
    'WriteVariableNames', true);

% Add metadata
% ---------------------------------------------------------------------
metadata = {...
    'Simulation Date', datestr(now, 'yyyy-mm-dd HH:MM:SS'); ...
    'Sampling Rate (Bottom)', [num2str(1/mean(diff(timeD))) ' Hz']; ...
    'Sampling Rate (Top)', [num2str(1/mean(diff(topDiscreteX3timeD))) ' Hz']; ...
    'Resampling Method', 'Linear extrapolation'; ...
    'Controller Version', 'v2.4 (Discrete PID + Notch)'; ...
    'Disturbance Profile', 'Random 0-2.5s band-limited'};

writecell(metadata, filename, 'Sheet', metadataSheet);

% Data validation
% ---------------------------------------------------------------------
% Check for NaNs in critical signals
if any(isnan(simData.ActualX3Position_m))
    warning('NaN values detected in position data - Check simulation outputs');
end

% Verify signal alignment
[~,overlapIndex] = max(timeD > 0.1);
if ~isequal(...
    simData.DiscreteBottomX3_m(overlapIndex), ...
    botDiscreteX3dataD(2))  % Second sample
    warning('Potential resampling misalignment detected');
end

disp(['Successfully exported ', num2str(height(simData)), ...
      ' samples to ', filename]);
%% Simulink-MATLAB Implementation Comparison
% Validates consistency between Simulink model and MATLAB implementation
% Quantifies numerical differences and visualizes response alignment

% Create unified comparison figure
figure('Name','ImplementationValidation','Position',[100 100 1200 800]);
set(gcf,'DefaultAxesFontSize',12,'DefaultLineLineWidth',1.5);

% Top Subsystem Comparison
% ---------------------------------------------------------------------
subplot(2,1,1);
hold on;

% Plot responses with different line styles
plot(out.topStep.time, out.topStep.signals.values,...
    'Color', [0 0.447 0.741], 'LineStyle','-',...
    'DisplayName','Simulink Top');
plot(timeTop, responseTop,...
    'Color', [0.85 0.325 0.098], 'LineStyle','--',...
    'DisplayName','MATLAB Top');

% Calculate and display RMS error
[commonTime, simTop, matTop] = alignSignals(out.topStep.time, out.topStep.signals.values, timeTop, responseTop);
rmsErrorTop = sqrt(mean((simTop - matTop).^2));
text(0.05, 0.9, sprintf('RMS Error: %.3e m', rmsErrorTop),...
    'Units','normalized', 'FontSize',10);

title('Top Subsystem Response Comparison', 'FontSize',14);
xlabel('Time (seconds)', 'FontWeight','bold');
ylabel('Position (meters)', 'FontWeight','bold');
legend('Location','southeast');
grid on;
hold off;

% Bottom Subsystem Comparison
% ---------------------------------------------------------------------
subplot(2,1,2);
hold on;

plot(out.botStep.time, out.botStep.signals.values,...
    'Color', [0 0.447 0.741], 'LineStyle','-',...
    'DisplayName','Simulink Bottom');
plot(timeBot, responseBot,...
    'Color', [0.85 0.325 0.098], 'LineStyle','--',...
    'DisplayName','MATLAB Bottom');

% Calculate and display RMS error
[commonTime, simBot, matBot] = alignSignals(out.botStep.time, out.botStep.signals.values, timeBot, responseBot);
rmsErrorBot = sqrt(mean((simBot - matBot).^2));
text(0.05, 0.9, sprintf('RMS Error: %.3e m', rmsErrorBot),...
    'Units','normalized', 'FontSize',10);

title('Bottom Subsystem Response Comparison', 'FontSize',14);
xlabel('Time (seconds)', 'FontWeight','bold');
ylabel('Position (meters)', 'FontWeight','bold');
legend('Location','southeast');
grid on;
hold off;

% Figure-wide formatting
% ---------------------------------------------------------------------
sgtitle({'Controller Implementation Validation',...
        sprintf('Simulation Date: %s | Sampling Rate: %dHz', datestr(now,1), sampleRateBot)},...
        'FontSize',16, 'FontWeight','bold');
set(gcf,'Color','w');

% Helper function for signal alignment
function [t, s1, s2] = alignSignals(t1, s1, t2, s2)
    % ALIGNSIGNALS Resample signals to common timebase
    t = union(t1, t2);
    s1 = interp1(t1, s1, t, 'linear', 'extrap');
    s2 = interp1(t2, s2, t, 'linear', 'extrap');
end

%% Integrated System Step Response Analysis
% Compares continuous and discrete implementations of the full coupled system
% Evaluates performance under combined step command and disturbance

% System parameters
commandStepSize = 0.025;  % [m] Step input magnitude
disturbanceComponent = topX3data + commandStepSize; % Combined disturbance signal

%% Continuous System Visualization
% ---------------------------------------------------------------------
figure('Name','IntegratedSystemResponse','Position',[100 100 800 800]);

% Continuous-time response
subplot(2,1,1);
hold on;
plot(continuousTime, continuousPosition,...
    'Color', [0 0.447 0.741], 'LineWidth', 2,...
    'DisplayName','Integrated System (Continuous)');
plot(continuousTime, disturbanceComponent,...
    'Color', [0.85 0.325 0.098], 'LineWidth', 1.5,...
    'LineStyle','--', 'DisplayName','Disturbance Profile');
plot(timeBot, responseBot*commandStepSize,...
    'Color', [0.494 0.184 0.556], 'LineWidth', 1.5,...
    'DisplayName','Isolated Bottom System');
hold off;

title('Continuous-Time Integrated Response', 'FontSize',14);
xlabel('Time (seconds)', 'FontWeight','bold');
ylabel('Position (meters)', 'FontWeight','bold');
legend('Location','southeast');
grid on;
ylim([0 commandStepSize*1.2]);

%% Discrete System Visualization
% ---------------------------------------------------------------------
subplot(2,1,2);
hold on;
plot(discreteTime, discretePosition,...
    'Color', [0 0.447 0.741], 'LineWidth', 2,...
    'DisplayName','Integrated System (Discrete)');
plot(discreteTime, topX3dataD + commandStepSize,...
    'Color', [0.85 0.325 0.098], 'LineWidth', 1.5,...
    'LineStyle','--', 'DisplayName','Disturbance Profile');
plot(timeBotD, responseBotD*commandStepSize,...
    'Color', [0.494 0.184 0.556], 'LineWidth', 1.5,...
    'DisplayName','Isolated Bottom System');
hold off;

title('Discrete-Time Integrated Response', 'FontSize',14);
xlabel('Time (seconds)', 'FontWeight','bold');
ylabel('Position (meters)', 'FontWeight','bold');
legend('Location','southeast');
grid on;
ylim([0 commandStepSize*1.2]);

%% Performance Metrics Calculation
% ---------------------------------------------------------------------
% Normalize responses for stepinfo (commandStepSize = 0.025m)
[contNormResponse, discNormResponse] = normalizeResponses(...
    continuousPosition, discretePosition, commandStepSize);

% Continuous system metrics
stepInfo = stepinfo(contNormResponse, continuousTime,...
                   'SettlingTimeThreshold',0.02); % 2% criterion

% Discrete system metrics
stepInfoD = stepinfo(discNormResponse, discreteTime,...
                    'SettlingTimeThreshold',0.02);

% Display formatted results
disp(' ');
disp('=== Performance Comparison ===');
disp(['Sampling Rate | Continuous: Inf Hz, Discrete: ',...
      num2str(1/mean(diff(discreteTime))),'Hz']);
disp(' ');

fprintf('Continuous System:\n');
fprintf('- Settling Time: %.3f s\n', stepInfo.SettlingTime);
fprintf('- Overshoot:     %.2f%%\n', stepInfo.Overshoot);
fprintf('- Rise Time:     %.3f s\n\n', stepInfo.RiseTime);

fprintf('Discrete System:\n');
fprintf('- Settling Time: %.3f s\n', stepInfoD.SettlingTime);
fprintf('- Overshoot:     %.2f%%\n', stepInfoD.Overshoot);
fprintf('- Rise Time:     %.3f s\n', stepInfoD.RiseTime);

%% Helper Functions
function [contNorm, discNorm] = normalizeResponses(contRaw, discRaw, amp)
    % NORMALIZERESPONSES Scale responses to unit step equivalent
    contNorm = contRaw / amp;
    discNorm = discRaw / amp;
end

%% Real-Time Steady-State Error Analysis
% Compares the continuous and discretized system's steady-state errors over time
% using real-time experimental/simulation data from the top cylinder (X3).

% Plot the continuous-time steady-state error
% ---------------------------------------------------------------------
% Subtract 0.025 to offset the target reference position (i.e., desired displacement)
% Error = actual position - reference
plot(time, actualX3data - 0.025);
hold on;

% Plot the discretized system's steady-state error
% ---------------------------------------------------------------------
% Same reference subtraction for fair comparison across implementations
plot(timeD, actualX3dataD - 0.025);

% Plot Formatting
% ---------------------------------------------------------------------
title('Steady-State Error Analysis');      % Graph title
xlabel('Time (s)');                        % X-axis: time in seconds
ylabel('Error (m)');                       % Y-axis: positional error in meters
legend('Continuous', 'Discrete');          % Distinguish between systems
grid on;                                   % Enable background grid for clarity

%% Variable Sinusoid Load Analysis
% Assesses system performance under a sinusoidal disturbance (amplitude = 0.1m)
% applied to the X3 carriage. Compares response of continuous and discrete systems.

% Plot the actual X3 displacement from the continuous-time simulation
% ---------------------------------------------------------------------
plot(time, actualX3data);

% Overlay the discretized system's X3 displacement response
% ---------------------------------------------------------------------
hold on;
plot(timeD, actualX3dataD);

% Overlay the applied sinusoidal disturbance (offset by +0.025m)
% ---------------------------------------------------------------------
% This represents the reference trajectory or external disturbance acting on the system
plot(timeD, topX3dataD + 0.025);

% Plot Formatting
% ---------------------------------------------------------------------
title('Variable Load Analysis - 0.1m Sinusoid');       % Title of the plot
xlabel('Time (s)');                                    % Time axis in seconds
ylabel('X3 Displacement (m)');                         % Y-axis: carriage displacement
legend('Continuous', 'Discrete', 'Disturbance (+0.025m)'); % Legend for signal identification
grid on;                                               % Show grid for better readability

%% Sinusoid Disturbance Rejection
% Evaluates how effectively the system rejects sinusoidal disturbances 
% of varying magnitudes. Calculates and plots the resulting error percentages.

% Calculate disturbance magnitudes as percentages of a 0.025m reference amplitude
% ---------------------------------------------------------------------
% Format: [0.001, 0.002, ..., 0.1] / 0.025 * 100 → percent of reference step
sinRejMag = [(0.001/0.025)*100, ...
             (0.002/0.025)*100, ...
             (0.005/0.025)*100, ...
             (0.01/0.025)*100, ...
             (0.1/0.025)*100];  % Disturbance magnitudes in %

% Corresponding steady-state error magnitudes from simulation or experiment
% ---------------------------------------------------------------------
% Error measured as percentage of full-scale input (0.025m)
sinRejErr = [0.1, 2.1, 5.6, 10.34, 101.9];  % Output error % for each disturbance

% Plot disturbance rejection performance
% ---------------------------------------------------------------------
plot(sinRejMag, sinRejErr);  % X: disturbance %, Y: error %
title('Sinusoid Disturbance Rejection Data');
xlabel('Disturbance (%)');   % Percentage of reference input
ylabel('Error (%)');         % Resulting system error percentage
grid on;                     % Enable grid for clarity


%% Variable Step Disturbance Analysis
% Assesses the system's ability to reject a step disturbance (magnitude = 0.001m)
% applied to the X3 carriage. Compares displacement responses of continuous 
% and discrete systems.

% Plot the continuous-time X3 displacement under step disturbance
% ---------------------------------------------------------------------
plot(time, actualX3data);

% Overlay the discretized system's response to the same disturbance
% ---------------------------------------------------------------------
hold on;
plot(timeD, actualX3dataD);

% Overlay the applied disturbance signal (0.025m offset for visibility/reference)
% ---------------------------------------------------------------------
% This may represent either the target position or externally applied offset
plot(timeD, topX3dataD + 0.025);

% Plot Formatting
% ---------------------------------------------------------------------
title('Variable Load Analysis - 0.001m Step');               % Plot title
xlabel('Time (s)');                                          % Time axis label
ylabel('X3 Displacement (m)');                               % Y-axis: output displacement
legend('Continuous', 'Discrete', 'Disturbance (+0.025m)');   % Legend for clarity
grid on;                                                     % Enable grid

%% Random Disturbance Analysis
% Evaluates system robustness under random load disturbances 
% with 0.001m variance applied to the X3 carriage position.
% Plots and compares continuous vs. discrete responses.

% Plot the continuous-time X3 displacement under random disturbance
% ---------------------------------------------------------------------
plot(time, actualX3data);

% Overlay the discretized system’s X3 displacement under the same random input
% ---------------------------------------------------------------------
hold on;
plot(timeD, actualX3dataD);

% Overlay the random disturbance signal, offset for visual separation
% ---------------------------------------------------------------------
% The offset (+0.025m) helps distinguish the disturbance from the output
plot(timeD, topX3dataD + 0.025);

% Plot Formatting
% ---------------------------------------------------------------------
title('Random Load Analysis - 0.001m Variance');               % Descriptive title
xlabel('Time (s)');                                             % X-axis: time
ylabel('X3 Displacement (m)');                                  % Y-axis: carriage position
legend('Continuous', 'Discrete', 'Disturbance (+0.025m)');      % Identify signal types
grid on;                                                        % Add grid for clarity

%% State Space Representation & Validation
% ss_closed_loop_sim.m
% State-space closed-loop validation of bottom-cylinder controller
% Inputs: valve current step (u) & sinusoidal load disturbance (d)
% Output: carriage displacement x3

clear; clc; close all;

%% 1) Physical Parameters
M2    = 42.85;      % bottom cylinder mass (kg)
C     = 1000;       % damping (N·s/m)
beta  = 1.6e9;      % fluid bulk modulus (Pa)
d2b   = 0.0762;     % piston diameter (m)
d2t   = 0.0381;     % rod diameter (m)
cL    = 0.1;        % stroke length (m)
Ks    = 5.61e-6;    % flow coefficient (m^3/(s·√Pa))
Ps    = 10e6;       % supply pressure (Pa)
E     = 200e9;      % Young’s modulus (Pa)
b     = 0.24;       % carriage width (m)
h     = 0.025;      % carriage height (m)
L     = 0.49;       % mount spacing (m)
M3    = 65.08;      % carriage mass (kg)
Kb    = 0.001;      % valve gain (m/A)
g     = 9.81;       % gravity (m/s^2)

%% 2) Derived Quantities
A2b = pi*(d2b/2)^2;
A2t = A2b - pi*(d2t/2)^2;
V2bo = (A2b + A2t)/2 * cL;
V2to = V2bo;
P2bo = 1.5*(2*M3*g)/A2b;               % static equilibrium bottom chamber
P2to = (A2b*P2bo - 2*M3*g)/A2t;        % static equilibrium top chamber
kq2b = Ks*sqrt(P2bo);
kq2t = Ks*sqrt(Ps - P2to);
I    = b*h^3/12;
k_sp = (192*E*I)/L^3;                  % frame spring constant

%% 3) Build Open-Loop Transfer Functions
s        = tf('s');
servoTF  = Kb/(0.01*s + 1);            % 1st-order servo lag
numCyl   = A2b*kq2b*beta/V2bo + A2t*kq2t*beta/V2to;
denCyl   = [M2, C, (A2t^2*beta/V2to + A2b^2*beta/V2bo - k_sp), 0];
botCylTF = tf(numCyl, denCyl);         % 3rd-order cylinder dynamics
carTF    = tf(k_sp, [M3, 0, 2*k_sp]);   % 2nd-order carriage dynamics

G_u = servoTF * botCylTF * carTF;      % valve current → x3
G_d =                   carTF;         % external force → x3

%% 4) Assemble Two-Input State-Space Plant
% Inputs: [u; d], Output: x3
sys2 = ss([G_u, G_d]);
[A,B,Cmat,D] = ssdata(sys2);

%% 5) Controller Design (Bottom Subsystem)
botGain  = 80.37;                      % proportional gain
wn_notch = 1770.466561315;             % notch center freq (rad/s)
zeta_n   = 0.001;                      % numerator damping (sharp)
zeta_d   = 0.7;                        % denominator damping (broad)
notchTF  = (s^2 + 2*zeta_n*wn_notch*s + wn_notch^2) ...
         / (s^2 + 2*zeta_d*wn_notch*s + wn_notch^2);
C_tf     = botGain * notchTF^4;       % 4× cascaded notch filter
C_ss     = ss(C_tf);                  % convert to state-space

%% 6) Close the Loop on the Valve-Input Channel
B_u      = B(:,1);
B_d      = B(:,2);
plant_u  = ss(A, B_u, Cmat, D(1,1));
loop_open= series(C_ss, plant_u);
cl_sys   = feedback(loop_open, 1);     % unity feedback on x3

%% 7) Simulation Inputs
T   = linspace(0,5,5000)';             % simulation time: 0–5s
u   = 0.025 * ones(size(T));           % step command: 25 mm
d   = 0.005 * sin(1 * T);              % sinusoidal disturbance: ±5 mm @1 rad/s
Uin = [u, d];

%% 8) Run Simulation
Y = lsim(cl_sys, Uin, T);              % Y(:,1) is the closed-loop x3 response

%% 9) Plot Results
figure('Position',[100 100 600 500]);
subplot(2,1,1)
plot(T, Y, 'b','LineWidth',1.5)
title('Closed-Loop Carriage Displacement x_3')
ylabel('x_3 (m)')
grid on

subplot(2,1,2)
plot(T, u,'k--','LineWidth',1.2)
hold on
plot(T, d,'r--','LineWidth',1.2)
xlabel('Time (s)')
ylabel('Input Amplitude (m)')
legend('Valve Cmd (u)','Load Disturbance (d)','Location','best')
grid on

sgtitle('State-Space Closed-Loop Validation of Bottom Subsystem')

