clear;
clc; 
syms u(t) a b phi
eqn = diff(u,t)== (-1/phi)*u+(1/phi)*(a*t+b);
uSolved = dsolve(eqn);
syms C1 C2 mu_s(t)
eqn2 = diff(mu_s,t) == -( (a/phi) - (C1/phi^2)*exp(-t/phi) + C2/phi);
muSolved = dsolve(eqn2);
syms lambda_v(t)


eqn3 = diff(lambda_v,t) == -C2-muSolved;
lambdaSolved = dsolve(eqn3)