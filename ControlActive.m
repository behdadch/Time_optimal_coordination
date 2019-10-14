function F = ControlActive(x,t0,tm,p0,pm,v0,vm,U)
  %Uncons. ==> ControlInputActive
% F(1) = 0.5*a*t0^2+b*t0+c-v0;
% F(2) = (1/6)*a*t0^3+0.5*b*t0^2+c*t0+d-p0;
% F(3) = U*tm+f-vm;
% F(4) = 0.5*U*tm^2+tm*f+g-pm;
% F(5) = 0.5*a*t1^2+b*t1+c-(U*t1+f);
% F(6) = (1/6)*a*t1^3+0.5*b*t1^2+c*t1+d-(0.5*U*t1^2+t1*f+g);
% F(7) = a*t1+b-U;

F(1) = 0.5*x(1)*tm^2+x(2)*tm+x(3)-vm;
F(2) = (1/6)*x(1)*tm^3+0.5*x(2)*tm^2+x(3)*tm+x(4)-pm;
F(3) = U*t0+x(5)-v0;
F(4) = 0.5*U*t0^2+t0*x(5)+x(6)-p0;
F(5) = 0.5*x(1)*x(7)^2+x(2)*x(7)+x(3)-(U*x(7)+x(5));
F(6) = (1/6)*x(1)*x(7)^3+0.5*x(2)*x(7)^2+x(3)*x(7)+x(4)-(0.5*U*x(7)^2+x(7)*x(5)+x(6));
F(7) = x(1)*x(7)+x(2)-U;


