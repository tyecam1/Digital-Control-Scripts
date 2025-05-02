% Define symbolic variables
syms m1 m2 I1 I2 l1 l2 lc1 lc2 g tau1 tau2 real
syms theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2 real

% Define position vectors for the centers of mass
p1 = lc1 * [cos(theta1); sin(theta1)]; % Position of center of mass of link 1
p2 = l1 * [cos(theta1); sin(theta1)] + lc2 * [cos(theta1 + theta2); sin(theta1 + theta2)]; % Position of center of mass of link 2

% Define velocities
v1 = diff(p1, theta1) * dtheta1;
v2 = diff(p2, theta1) * dtheta1 + diff(p2, theta2) * dtheta2;

% Kinetic energies
T1 = (1/2) * m1 * sum(v1.^2) + (1/2) * I1 * dtheta1^2;
T2 = (1/2) * m2 * sum(v2.^2) + (1/2) * I2 * (dtheta1 + dtheta2)^2;
T = T1 + T2;

% Potential energies
V1 = m1 * g * lc1 * cos(theta1);
V2 = m2 * g * (l1 * cos(theta1) + lc2 * cos(theta1 + theta2));
V = V1 + V2;

% Inertia matrix D(q)
D = [diff(diff(T, dtheta1), dtheta1), diff(diff(T, dtheta1), dtheta2); 
     diff(diff(T, dtheta2), dtheta1), diff(diff(T, dtheta2), dtheta2)];

% Gravity vector G(q)
G = [diff(V, theta1); diff(V, theta2)];

% Coriolis and centrifugal matrix C(q, q_dot)
C = sym(zeros(2));
for i = 1:2
    for j = 1:2
        for k = 1:2
            C(i, j) = C(i, j) + 1/2 * (diff(D(i, j), ['theta', num2str(k)]) + ...
                      diff(D(i, k), ['theta', num2str(j)]) - ...
                      diff(D(j, k), ['theta', num2str(i)])) * ['dtheta', num2str(k)];
        end
    end
end

% Display the results
disp('Inertia matrix D(q):')
disp(simplify(D))

disp('Coriolis and centrifugal matrix C(q, q_dot):')
disp(simplify(C))

disp('Gravity vector G(q):')
disp(simplify(G))
