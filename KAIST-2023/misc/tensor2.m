syms a b c rho

% mass of web and flange
m = rho * a * b * c;

% inertia moments
IxxWeb = 1/12 * m * (b^2 + c^2);
IyyWeb = 1/12 * m * (a^2 + b^2);
IzzWeb = 1/12 * m * (a^2 + c^2);

IxxFlange = IxxWeb;
Iyyflange = 1/12 * m * (a^2 + c^2);
Izzflange = 1/12 * m * (a^2 + b^2);


% parallel axis theorem
IxxFlangeFinal = IxxFlange + m * (0.5*b+0.5*c)^2;
IyyFlangeFinal = Iyyflange + m * (0.5*b+0.5*c)^2;
% z goes through the centre

IxxTotal = simplify(IxxWeb + 2 * IxxFlangeFinal);
IyyTotal = simplify(IyyWeb + 2 * IyyFlangeFinal);
IzzTotal = simplify(IzzWeb + 2 * Izzflange);

% Inertia Tensor
Inertia_Tensor = [IxxTotal, 0, 0; 0, IyyTotal, 0; 0, 0, IzzTotal]
