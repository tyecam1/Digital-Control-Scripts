

syms alpha
syms beta
syms gamma
% INPUT:
% alpha, beta, gamma - Euler angles (ZYZ convention)

% Calculate the elements of the Jacobian matrix based on the ZYZ convention
J_rot = [
    0, sin(alpha)*sin(beta), cos(alpha)*sin(beta);
    0, cos(alpha)*cos(beta), -sin(alpha)*cos(beta);
    1, sin(alpha), cos(alpha)]

% Modify the Jacobian matrix by inserting the specific values of the angles
J_rot = subs(J_rot, [alpha, beta, gamma]);

