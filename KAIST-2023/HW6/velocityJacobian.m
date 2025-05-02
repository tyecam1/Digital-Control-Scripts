% vars & table
syms theta1 theta2 theta3
% [theta, d, a, alpha]
table = [theta1+pi/2, 0.5, 0, pi/2;
            theta2, 0, 1.5, 0;
            theta3,0,1,0];
joints = [theta1; theta2; theta3];


% Jacobian
Jv = simplify(velocity_jacobian(table, joints))


function Jv = velocity_jacobian(table, joints)    
    N = size(table, 1); % number of joints for future use
    Jv = sym(zeros(3, N)); % initializing jacobian matrix as symbolic
    
    for i = 1:N
        theta = joints(i);
        d = table(i, 2);
        a = table(i, 3);
        alpha = table(i, 4);
        
        % H transformation matrix from the previous frame to the current frame
        A_i = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
               sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
               0, sin(alpha), cos(alpha), d;
               0, 0, 0, 1];
           
        % calculate final transform
        if i == 1
            T = A_i;
        else
            T = T * A_i;
        end
        
        % rotation matrix
        R = T(1:3, 1:3);
        
        % find z-axis
        if i == 1
            z = [0; 0; 1];
        else
            z = R_prev(:, 3);
        end
        
        % calculate p0n
        p0n = T(1:3, 4);
        
        % calculate velocity jacobian columns
        Jv(:, i) = cross(z, p0n);
        
        R_prev = R; % store rotation matrix for the next iteration
    end
end
