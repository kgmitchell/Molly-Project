function l = IK(P)
o = P;
u1 = [-100 0]';
u2 = [100 0]';
u = [u1 u2];

for i = 1:2
    L(:,i) = o - u(:,i);
    l(i) = norm(L(:,i),2);
end

