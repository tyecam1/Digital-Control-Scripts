syms a b c rho y z x

% Ixx

webLimitsX = [0, a];
webLimitsY = [-b/2, b/2];
webLimitsZ = [0, c];

flangeLimitsX = [0, a];
% two sets for top and bottom
flangeLimitsY = [b/2, b/2 + c;-b/2 - c, -b/2];
flangeLimitsZ = [0, c];

IxxWeb = int(int(int(y^2 * rho, y, webLimitsY(1), webLimitsY(2)), z, webLimitsZ(1), webLimitsZ(2)), x, webLimitsX(1), webLimitsX(2));
IxxFlangeTop = int(int(int(y^2 * rho, y, flangeLimitsY(1,1), flangeLimitsY(1,2)), z, flangeLimitsZ(1), flangeLimitsZ(2)), x, flangeLimitsX(1), flangeLimitsX(2));
IxxFlangeBottom = int(int(int(y^2 * rho, y, flangeLimitsY(2,1), flangeLimitsY(2,2)), z, flangeLimitsZ(1), flangeLimitsZ(2)), x, flangeLimitsX(1), flangeLimitsX(2));

IxxTotal = simplify(IxxWeb + IxxFlangeTop + IxxFlangeBottom)

% Iyy

webLimitsXyy = [0, a];
webLimitsYyy = [-b/2, b/2];
webLimitsZyy = [0, c];

flangeLimitsXyy = [0, a];
% Two sets for top and bottom
flangeLimitsYyy = [b/2, b/2 + c; -b/2 - c, -b/2];
flangeLimitsZyy = [0, c];

IyyWeb = int(int(int(z^2 * rho, z, webLimitsZyy(1), webLimitsZyy(2)), y, webLimitsYyy(1), webLimitsYyy(2)), x, webLimitsXyy(1), webLimitsXyy(2));
IyyFlange = int(int(int(z^2 * rho, z, flangeLimitsZyy(1), flangeLimitsZyy(2)), y, flangeLimitsYyy(1,1), flangeLimitsYyy(1,2)), x, flangeLimitsXyy(1), flangeLimitsXyy(2)) + ...
              int(int(int(z^2 * rho, z, flangeLimitsZyy(1), flangeLimitsZyy(2)), y, flangeLimitsYyy(2,1), flangeLimitsYyy(2,2)), x, flangeLimitsXyy(1), flangeLimitsXyy(2));

IyyTotal = simplify(IyyWeb + IyyFlange)

% Izz

webLimitsXzz = [0, a];
webLimitsYzz = [-b/2, b/2];
webLimitsZzz = [0, c];

flangeLimitsXzz = [0, a];
flangeLimitsYzz = [-c/2, c/2];
flangeLimitsZzz = [0, c];

IzzWeb = int(int(int(x^2 * rho, x, webLimitsXzz(1), webLimitsXzz(2)), y, webLimitsYzz(1), webLimitsYzz(2)), z, webLimitsZzz(1), webLimitsZzz(2));
IzzFlange = 2 * int(int(int(x^2 * rho, x, flangeLimitsXzz(1), flangeLimitsXzz(2)), y, flangeLimitsYzz(1), flangeLimitsYzz(2)), z, flangeLimitsZzz(1), flangeLimitsZzz(2));

IzzTotal = simplify(IzzWeb + IzzFlange)

