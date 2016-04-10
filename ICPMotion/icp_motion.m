clear all; clc;


x = [1 2 3 4 5 10]
y = [5 6 7 8 9 10]
z = [5 2 7 2 1 10]

syms x xv y yv vx vy ax ay t1 t2 t3 t4
syms dx1 dx2 dx3 dy1 dy2 dy3 dx4 dy4
syms a b c d e f

% Constant velocity
% f = (x-xv+vx*t1)^2+(y-yv+vy*t1)^2 + ...
%     (x-xv+vx*t2)^2+(y-yv+vy*t2)^2 + ...
%     (x-xv+vx*t3)^2+(y-yv+vy*t3)^2 + ...
%     (x-xv+vx*t4)^2+(y-yv+vy*t4)^2;
% 
% dfvx = diff(f, vx)
% dfvy = diff(f, vy)
% 
% eqs = [dfvx, dfvy];
% res = solve(eqs, [vx, vy]);
% res.vx
% res.vy

% Constant acceleration

f = (dx1/t1+vx+ax*t1)^2+(dy1/t1+vy+ay*t1)^2;

% f = (dx1+vx*t1+ax/2*t1^2)^2+(dy1+vy*t1+ay/2*t1^2)^2 +  ...
%     (dx2+vx*t2+ax/2*t2^2)^2+(dy2+vy*t2+ay/2*t2^2)^2;


% f = (dx1+vx*t1+ax/2*t1^2)^2+(dy1+vy*t1+ay/2*t1^2)^2 + ...
%     (dx2+vx*t2+ax/2*t2^2)^2+(dy2-yv+vy*t2+ay/2*t2^2)^2 + ...
%     (dx3+vx*t3+ax/2*t3^2)^2+(dy3-yv+vy*t3+ay/2*t3^2)^2 + ...
%     (dx4+vx*t4+ax/2*t4^2)^2+(dy4-yv+vy*t4+ay/2*t4^2)^2;

% f = (x-xv+vx*a+ax/2*d)^2+(y-yv+vy*a+ay/2*d)^2 + ...
%     (x-xv+vx*b+ax/2*e)^2+(y-yv+vy*b+ay/2*e)^2 + ...
%     (x-xv+vx*c+ax/2*f)^2+(y-yv+vy*c+ay/2*f)^2;


dfvx = diff(f, vx)
dfax = diff(f, ax)
dfvy = diff(f, vy);
dfay = diff(f, ay);

eqs = [dfvx, dfax, dfvy, dfay];
res = solve(eqs, [vx, ax, vy, ay]);
res.vx
res.ax

simplify(res.vx)
simplify(res.ax)

return;
