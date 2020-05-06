function f= fuel(v,u)
%based on the model presented in 
%Kamal et al. Model Predictive Control of Vehicles on Urban Roads for Improved Fuel Economy
% ml/s 
%Question: Should we consider 
b=[0.1569 2.45*10^(-2) -7.415*10^(-4) 5.975*10^(-5)];
c=[0.07224 9.681*10^(-2) 1.075*10^(-3)];
M=1200;
Cd=0.32;
ro=1.184;
Av=2.5;
mu=0.015;
g=9.81;
a_m= ((-1/(2*M))*Cd*ro*Av*v.^2)-mu*g;
a_hat = u +a_m;
f_cr = b(1)+b(2).*v+b(3).*v.^2+b(4).*v.^3;
f_acc = a_hat.*(c(1)+c(2).*v+c(3).*v.^2);
f=f_cr+f_acc;
end