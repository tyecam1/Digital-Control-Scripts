function main
    % Time vector
    t = 0:0.001:1;
    tau = zeros(length(t), 2);

    % System Parameters (define these according to your robot's specification)
    g = 9.81; m1 = 1; m2 = 1; l1 = 1; l2 = 1; lc1 = 0.5; lc2 = 0.5; I1 = 0.1; I2 = 0.1;

    % Noise
    noise1 = randn(1, length(t)) * 0.01;
    noise2 = randn(1, length(t)) * 0.01;

    for i = 1:length(t)
        tau_sym = inverseDynamics(t(i), noise1(i), noise2(i), m1, m2, l1, l2, lc1, lc2, I1, I2, g);
        tau(i, :) = double(vpa(tau_sym, 6));  % Convert symbolic expression to double
    end

    % Plotting the results
    plot(t, tau);
    xlabel('Time (s)');
    ylabel('Torque (Nm)');
    legend('Joint 1', 'Joint 2');
    title('Joint Torques over Time');
end

function tau_sym = inverseDynamics(currentTime, noise1, noise2, m1, m2, l1, l2, lc1, lc2, I1, I2, g)
    % Define the symbolic variables
    syms theta1 theta2 dtheta1 dtheta2 ddtheta1 ddtheta2

    % Joint angles and their derivatives
    q = [theta1; theta2];
    dq = [dtheta1; dtheta2];
    ddq = [ddtheta1; ddtheta2];

    q1 = (pi/4) + cos(currentTime) + noise1;
    q2 = (pi/4) + cos(2*currentTime) + noise2;
    dq1 = -sin(currentTime);  
    dq2 = -2*sin(2*currentTime);
    ddq1 = -cos(currentTime);
    ddq2 = -4*cos(2*currentTime);

    % Position vectors of the centers of mass
    p1 = [lc1 * cos(theta1); lc1 * sin(theta1)];
    p2 = [l1 * cos(theta1) + lc2 * cos(theta1 + theta2); l1 * sin(theta1) + lc2 * sin(theta1 + theta2)];

    % Velocities of the centers of mass
    v1 = diff(p1, theta1) * dtheta1 + diff(p1, theta2) * dtheta2;
    v2 = diff(p2, theta1) * dtheta1 + diff(p2, theta2) * dtheta2;

    % Kinetic energies
    T1 = (1/2) * m1 * (transpose(v1) * v1) + (1/2) * I1 * dtheta1^2;
    T2 = (1/2) * m2 * (transpose(v2) * v2) + (1/2) * I2 * (dtheta1 + dtheta2)^2;

    % Potential energies
    V1 = m1 * g * lc1 * sin(theta1);
    V2 = m2 * g * (l1 * sin(theta1) + lc2 * sin(theta1 + theta2));

    % Inertia matrix M(q)
    M = [diff(diff(T1 + T2, dtheta1), dtheta1), diff(diff(T1 + T2, dtheta1), dtheta2); 
         diff(diff(T1 + T2, dtheta2), dtheta1), diff(diff(T1 + T2, dtheta2), dtheta2)];

    % Coriolis and centrifugal matrix C(q, dq)
    C = sym(zeros(2,2));
    for i = 1:2
        for j = 1:2
            C(i,j) = 0;
            for k = 1:2
                C(i,j) = C(i,j) + 1/2 * (diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(j,k), q(i))) * dq(k);
            end
        end
    end

    % Gravity vector G(q)
    G = [diff(V1 + V2, theta1); diff(V1 + V2, theta2)];

    % Substitute actual values
    tau_sym = subs(simplify(M * ddq + C * dq + G), {theta1, theta2, dtheta1, dtheta2, ddtheta1, ddtheta2}, {q1, q2, dq1, dq2, ddq1, ddq2});
end
