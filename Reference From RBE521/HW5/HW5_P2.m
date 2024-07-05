% Kyle Mitchell
% HW5 Problem 2

%to maximuxe v, (1-beta/beta) must be maximized. Having beta value very
%close to zero would maximize this value. Therefore, the duty factor to
%maximize robot speed v is beta > 0 and beta is very close to 0.

beta = 0:0.05:1;

u_t = rand; % 0 m/s to 1 m/s (u_t is known but arbitrary)

for i = 1:size(beta,2)
        v_t(i) = ((1-beta(i))/(beta(i))) * u_t; 
end

figure
plot(beta,v_t)
legend
    