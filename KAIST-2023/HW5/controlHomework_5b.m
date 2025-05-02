% Define symbolic variables
syms m1 m2 I1 I2 l1 l2 g
syms theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2

% Define vectors of symbolic variables
q = [theta1; theta2];
dq = [dtheta1; dtheta2];
ddq = [ddtheta1; ddtheta2];

% Inertia matrix M(q)
M = [I1 + I2 + (l1^2*m1)/4 + l1^2*m2 + (l2^2*m2)/4 + l1*l2*m2*cos(theta2 + 90), (m2*l2^2)/4 + (l1*m2*cos(theta2 + 90)*l2)/2 + I2;
     (m2*l2^2)/4 + (l1*m2*cos(theta2 + 90)*l2)/2 + I2,                                 (m2*l2^2)/4 + I2];

% Coriolis and centrifugal matrix C(q, dq)
C = [-dtheta2*l1*l2*m2*sin(theta2 + 90)/2, -(l1*l2*m2*sin(theta2 + 90)*(dtheta1 + dtheta2))/2;
      dtheta1*l1*l2*m2*sin(theta2 + 90)/2,                                                  0];

% Gravity vector G(q)
G = [g*m2*((l2*cos(theta1 + theta2 + 90))/2 + l1*cos(theta1)) + (g*l1*m1*cos(theta1))/2;
     (g*l2*m2*cos(theta1 + theta2 + 90))/2];

% Define the inertial parameter vector
phi = [m1; m2; I1; I2];

% Construct the regressor matrix Y
% The exact expressions depend on the specifics of your system
% Placeholder for Y
Y = sym(zeros(2, length(phi)));

% Example of filling in the Y matrix
% Y(:, 1) = expressions involving q, dq, ddq that correspond to m1 in M, C, G
% Y(:, 2) = expressions involving q, dq, ddq that correspond to m2 in M, C, G
% Y(:, 3) = expressions involving q, dq, ddq that correspond to I1 in M, C, G
% Y(:, 4) = expressions involving q, dq, ddq that correspond to I2 in M, C, G

% Fill in the expressions for Y based on the specific dynamics of your system
% For example:
% Y(:, 1) might contain terms from M, C, G that linearly depend on m1
% Similarly for m2, I1, and I2

% Display the results
disp('Regressor matrix Y(q, dq, ddq):')
disp(simplify(Y))
