function F = UmaxUnconUmin(x,t0,tm,p0,pm,v0,vm,U1,U2)
  %Umax (e,f) t1==> Uncons.(a,b,c,d) t2==> Umin(g,h)
    % e=x1,f=x2, t1=x3    a=x4 b=x5 c=x6 d=x7   t2=x8 g=x9  h=x10
    
    
    
F(1) = U1*t0+x(1)-v0;
F(2) = 0.5*U1*t0^2+t0*x(1)+x(2)-p0;

F(3) = 0.5*x(4)*x(3)^2+x(5)*x(3)+x(6)-(U1*x(3)+x(1));
F(4) = (1/6)*x(4)*x(3)^3 + 0.5*x(5)*x(3)^2 + x(6)*x(3) + x(7)-(0.5*U1*x(3)^2+x(3)*x(1)+x(2));
F(5) = x(4)*x(3)+x(5)-U1;

F(6) = 0.5*x(4)*x(8)^2+x(5)*x(8)+x(6)-(U2*x(8)+x(9));
F(7) = (1/6)*x(4)*x(8)^3 + 0.5*x(5)*x(8)^2 + x(6)*x(8) + x(7)-(0.5*U2*x(8)^2+x(9)*x(8)+x(10));
F(8) = x(4)*x(8)+x(5)-U2;

F(9) = U2*tm+x(9)-vm;
F(10) = 0.5*U2*tm^2+tm*x(9)+x(10)-pm;

