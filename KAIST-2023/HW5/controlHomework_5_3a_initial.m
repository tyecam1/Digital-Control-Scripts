% Define symbolic variables
syms m1 m2 I1 I2 l1 l2 g real
syms theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2

% Define vectors of symbolic variables
theta = [theta1; theta2];
dtheta = [dtheta1; dtheta2];
ddtheta = [ddtheta1; ddtheta2];

% Define position vectors for the centers of mass
p1 = l1 * 0.5 * [cos(theta1); sin(theta1)]; % Position of center of mass of link 1
p2 = l1 * [cos(theta1); sin(theta1)] + l2 * 0.5 * [cos(theta1 + theta2); sin(theta1 + theta2)]; % Position of center of mass of link 2

% Define velocities
v1 = diff(p1, theta1) * dtheta1;
v2 = diff(p2, theta1) * dtheta1 + diff(p2, theta2) * dtheta2;

% Kinetic energies
T1 = (1/2) * m1 * sum(v1.^2) + (1/2) * I1 * dtheta1^2;
T2 = (1/2) * m2 * sum(v2.^2) + (1/2) * I2 * (dtheta1 + dtheta2)^2;
T = T1 + T2;

% Potential energies
V1 = m1 * g * p1(2);
V2 = m2 * g * p2(2);
V = V1 + V2;

% Inertia matrix M(q)
M = sym(zeros(2));
for i = 1:2
    for j = 1:2
        M(i, j) = diff(diff(T, dtheta(j)), theta(i));
    end
end

% Gravity vector G(q)
G = sym(zeros(2, 1));
for i = 1:2
    G(i) = diff(V, theta(i));
end

% Coriolis and centrifugal matrix C(q, q_dot)
C = sym(zeros(2));
for i = 1:2
    for j = 1:2
        C(i, j) = 0;
        for k = 1:2
            C(i, j) = C(i, j) + 1/2*(diff(M(i, j), theta(k)) + diff(M(i, k), theta(j)) - diff(M(j, k), theta(i))) * dtheta(k);
        end
    end
end

% Display the results
disp('Inertia matrix M(q):')
disp(simplify(M))

disp('Coriolis and centrifugal matrix C(q, q_dot):')
disp(simplify(C))

disp('Gravity vector G(q):')
disp(simplify(G))
