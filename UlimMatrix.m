function[a,b,c,d,f,g,t1]= UlimMatrix(t_0,t_m,p_0,p_m,v_0,v_m,u_min)
% F(1) = 0.5*a*t0^2+b*t0+c-v0;
% F(2) = (1/6)*a*t0^3+0.5*b*t0^2+c*t0+d-p0;
% F(3) = umin*tm+f-vm;
% F(4) = 0.5*umin*tm^2+tm*f+g-pm;
% F(5) = 0.5*a*t1^2+b*t1+c-(umin*t1+f);
% F(6) = (1/6)*a*t1^3+0.5*b*t1^2+c*t1+d-(0.5*umin*t1^2+t1*f+g);
% F(7) = a*t1+b-umin;
f = v_m - u_min*t_m;
g = -0.5*u_min*t_m^2-t_m*f+p_m;
t1 = (6*p_m - 6*p_0 + 2*t_0*v_0 + 4*t_0*v_m - 6*t_m*v_m + t_0^2*u_min + 3*t_m^2*u_min - 4*t_0*t_m*u_min)/(2*(v_0 - v_m - t_0*u_min + t_m*u_min)); %coming from symbolic solver

p_1 = 0.5*u_min*t1^2+f*t1+g;
v_1 = u_min*t1+f;


A =[t_0^3/6 t_0^2/2 t_0 1; t_0^2/2 t_0 1 0; t1^3/6 t1^2/2 t1 1; t1^2/2 t1 1 0];
Y =[p_0; v_0; p_1; v_1];
X = A\Y; 
a=X(1);
b=X(2);
c=X(3);
d=X(4);
end