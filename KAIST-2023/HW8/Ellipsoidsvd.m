T1 = deg2rad(0);
T2 = deg2rad(60);
T3 = deg2rad(78.5904);
L2 = 1.5;
L3 = 1;

% jacobian 
J = [- (L2 * cos(T2) + L3 * cos(T2+T3)) * cos(T1),  (L2 * sin(T2) + L3 * sin(T2+T3)) * sin(T1), L3 * sin(T2+T3) * sin(T1);
              - (L2 * cos(T2) + L3 * cos(T2+T3)) * sin(T1), - (L2 * sin(T2) + L3 * sin(T2+T3)) * cos(T1), - L3 * sin(T2+T3) * cos(T1);
               0, L2 * cos(T2) + L3 * cos(T2+T3), L3 * cos(T2+T3)];

J*transpose(J)

% singular decomposition
[U, S, V] = svd(J);
a = S(1,1);
b = S(2,2);
c = S(3,3);

% singular sphere to represent q*q*T
[x,y,z] = sphere;

% scale the sphere to velocity ellipsoid
ell_x = a*x;
ell_y = b*y;
ell_z = c*z;

% rotate ellispoid using the third collumn vector of SVD func
rotated_x = zeros(size(ell_x));
rotated_y = zeros(size(ell_y));
rotated_z = zeros(size(ell_z));

for i = 1:numel(ell_x)
    rotated_point = V * [ell_x(i); ell_y(i); ell_z(i)];
    rotated_x(i) = rotated_point(1);
    rotated_y(i) = rotated_point(2);
    rotated_z(i) = rotated_point(3);
end

figure(1)
surf(rotated_x, rotated_y, rotated_z);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('velocity manipulability ellipsoid 2');