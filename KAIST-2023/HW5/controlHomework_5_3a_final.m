
syms m1 m2 I1 I2 l1 l2 g
syms theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2

q = [theta1; theta2];
dq = [dtheta1; dtheta2];
ddq = [ddtheta1; ddtheta2];

% CoMs
p1 = [l1/2 * cos(theta1); l1/2 * sin(theta1)];
p2 = [l1 * cos(theta1) + l2/2 * cos(theta2+pi*0.5); l1 * sin(theta1) + l2/2 * sin(theta2+pi*0.5)];

% velocities
v1 = jacobian(p1, q) * dq;
v2 = jacobian(p2, q) * dq;

% kinetic energiy
T1 = (1/2) * m1 * v1.' * v1 + (1/2) * I1 * dtheta1^2;
T2 = (1/2) * m2 * v2.' * v2 + (1/2) * I2 * (dtheta1 + dtheta2)^2;
T = T1 + T2;

% potential energy
V1 = m1 * g * p1(2);
V2 = m2 * g * p2(2);
V = V1 + V2;

% inertia matrix M(q)
M = [diff(diff(T, dtheta1), dtheta1), diff(diff(T, dtheta1), dtheta2); 
     diff(diff(T, dtheta2), dtheta1), diff(diff(T, dtheta2), dtheta2)];

% coriolis matrix C(q, dq)
C = sym(zeros(2,2));
for i = 1:2
    for j = 1:2
        C(i,j) = 0;
        for k = 1:2
            C(i,j) = C(i,j) + 1/2 * (diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(j,k), q(i))) * dq(k);
        end
    end
end

% gravity G(q)
G = [diff(V, theta1); diff(V, theta2)];

disp('M(q):')
disp(simplify(M))

disp('C(q, dq):')
disp(simplify(C))

disp('G(q):')
disp(simplify(G))
