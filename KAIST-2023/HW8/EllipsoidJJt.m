

% using JJT
JJt = J * transpose(J);

[V, D] = eig(JJt);

% axis lengths
a = sqrt(D(1,1));
b = sqrt(D(2,2));
c = sqrt(D(3,3));

% q*q*t
[x, y, z] = sphere;

% scale the sphere to the size of the ellipsoid
ell_x = a * x;
ell_y = b * y;
ell_z = c * z;

% rotate the ellipsoid
rotated_x = zeros(size(ell_x));
rotated_y = zeros(size(ell_y));
rotated_z = zeros(size(ell_z));

for i = 1:numel(ell_x)
    rotated_point = V * [ell_x(i); ell_y(i); ell_z(i)];
    rotated_x(i) = rotated_point(1);
    rotated_y(i) = rotated_point(2);
    rotated_z(i) = rotated_point(3);
end

% plot the ellipsoid
figure(2);
surf(rotated_x, rotated_y, rotated_z);
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Velocity Manipulability Ellipsoid');