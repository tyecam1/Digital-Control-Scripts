% joint angles
q = [60; 0; 0; 0];
q = deg2rad(q);

% link lengths
l1 = 0.3;
l2 = 0.2;
l3 = 0.2;
l4 = 0.1;

% calculate j
J = zeros(2,4);
J(1,1) = -l1*sin(q(1)) - l2*sin(q(1)+q(2)) - l3*sin(q(1)+q(2)+q(3)) - l4*sin(q(1)+q(2)+q(3)+q(4));
J(1,2) = -l2*sin(q(1)+q(2)) - l3*sin(q(1)+q(2)+q(3)) - l4*sin(q(1)+q(2)+q(3)+q(4));
J(1,3) = -l3*sin(q(1)+q(2)+q(3)) - l4*sin(q(1)+q(2)+q(3)+q(4));
J(1,4) = -l4*sin(q(1)+q(2)+q(3)+q(4));
J(2,1) = l1*cos(q(1)) + l2*cos(q(1)+q(2)) + l3*cos(q(1)+q(2)+q(3)) + l4*cos(q(1)+q(2)+q(3)+q(4));
J(2,2) = l2*cos(q(1)+q(2)) + l3*cos(q(1)+q(2)+q(3)) + l4*cos(q(1)+q(2)+q(3)+q(4));
J(2,3) = l3*cos(q(1)+q(2)+q(3)) + l4*cos(q(1)+q(2)+q(3)+q(4));
J(2,4) = l4*cos(q(1)+q(2)+q(3)+q(4))

I = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,1];

% eq for P
P = I - pinv(J)*J

% calculate the rank and nullity of J
rank_J = rank(J)
nullity_J = 4 - rank_J

% rank of P
rank_P = rank(P)

