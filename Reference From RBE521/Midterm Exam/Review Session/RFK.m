function p = RFK(lg)
u1 = [-107 6]';
u2 = [95 3]';
u = [u1 u2];
c = norm(u2 - u1);

lo_nominal = [200 200];
lo_real = [204 197];
lo_error = lo_real - lo_nominal;
a = lg(1) + lo_error(1);
b = lg(2) + lo_error(2);





D = (a^2 + c^2 - b^2)/(2 * a * c);
A1 = acos(D);
U1 = atan(abs(u1(2)/u1(1)));
AU = acos(((norm(u1))^2 + c^2 - (norm(u2))^2)/(2 * norm(u1) * c));
A1R = A1 - (U1 - AU);

L1 = [a * cos(A1R)   a * sin(A1R)]';

p = u1 + L1;
