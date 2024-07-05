function F = CF(x)
u1t = [-100 0]';
u2t = [100 0]';
ut = [u1t u2t];

pt = [
   -50 250
    50 250
     75 230
    -75 230
    -90 210
    90 210
];

m = 3;

for i = 1:m
    pm(i , 1:2) = (RFK(IK(pt(i,:)')))';
end

j = 0;
for k = 1:m
    for i= 1:2
        j = j+1;
        L(:,i) = (pt(k,1:2))' - ut(:,i); % Nominal IK
        Ltest(:,i) = IK(pt(k,1:2)');
        l(i) = norm(L(:,i),2); % Nominal leg lengths
        lo = IK([0 173.2051]') ; %Nominal Initial Leg Lengths
        F(j) = ((norm([pm(k,1:2)]' - x(1:2,i)))^2 - norm(x(3,i) + l(i) - lo(i))^2)^2;
    end
end

