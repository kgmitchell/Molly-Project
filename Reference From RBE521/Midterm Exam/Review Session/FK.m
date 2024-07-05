function p = FK(lg)
u1 = [-100 0]';
u2 = [100 0]';
u = [u1 u2];
c = norm(u2-u1);
a = lg(1);
b = lg(2);

D = (a^2 + c^2 - b^2)/(2 * a * c);
A1 = acos(D);

L1 = [a * D    a * sin(A1)]';

p = u1 + L1;