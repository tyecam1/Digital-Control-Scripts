
% Problem 1
% a)
%v = [2,0,2,3,6,2,8,2]

% Problem 2
% a)
%x1 = [1/sqrt(2),1/sqrt(2),0];
%x2 = [-1/2,1/2,1/sqrt(2)];
%x3 = [1/2,-1/2,1/sqrt(2)];

%M = [x1;x2;x3];

%Mt = M*M';

% b)

%A = [3,5,1;7,2,4;5,5,1];
%b = [8;-3;2];

%xInverse = inv(A)*b
%xBackslash = A\b

% Problem 3
%m = 7
%k = 3
%b = 2

% Problem 4
% b)
i = 0
for t = 0:0.1:10
    i = i+1;
    s(i) = displacement_20236282(t);
end
disp("The final displacement at 10 seconds is "+num2str(s(i))+"m")
plot(0:0.1:10,s,"-blue")
xlabel("Time")
ylabel("Displacement")
title("Displacement over Time")