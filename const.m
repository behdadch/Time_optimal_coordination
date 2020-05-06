clc
clear
% syms u_max u_min v_max t0 t1 t2 tf vStart pStart Pt1 Pt2 pEnd vEnd
% t0=0;
%  t1 = (v_max-vStart)/u_max+t0;
%  Pt1 = 0.5*u_max*(t1^2-t0^2)-u_max*t0*(t1-t0)+vStart*(t1-t0)+ pStart;
% f1 = 0.5*u_min*(tf^2-t2^2)-u_min*t2*(tf-t2)+v_max*(tf-t2)+v_max*(t2-t1)+Pt1;
% f3 = 0.5*u_min*(tf^2-t2^2)-u_min*t2*(tf-t2)+v_max*(tf-t2)+Pt2;
% f2 = u_min*(tf-t2)+v_max;
% [sol.tf, sol.t2] = solve(f1-pEnd==0,f2-vEnd==0,tf,t2);
% sol.tf
%%
% syms u_max u_min v_max t T t1 t2 tf vStart pStart Pt1 Vt1 pEnd vEnd a b c d
% f1 =  0.5*u_min*(t1^2-T^2) - u_min*T*(t1-T) + vStart*(t1-T) + pStart;
% f2 = (1/6)*a*(t1)^3+0.5*b*(t1)^2+c*(t1)+d;
% f3 =  u_min*(t1-T)+vStart;
% f4 =  0.5*a*(t1)^2+b*(t1)+c;
% f5 = -pEnd + (1/6)*a*(tf)^3+0.5*b*(tf)^2+c*(tf)+d;
% f6 = -vEnd + 0.5*a*(tf)^2+b*(tf)+c;
% [sol.t1,a,b,c,d] = solve(f1==f2,f3==f4,f5==0,f6==0,t1,a,b,c,d);
% syms u_min u_max t_s t_c t_e p_s p_c p_e v_s v_c v_e 
% f1 = u_min*(t_c-t_s) + v_s;
% f2 =  0.5*u_min*(t_c^2-t_s^2)-u_min*t_s*(t_c-t_s)+v_s*(t_c-t_s)+ p_s;
% f3 = u_max*(t_e-t_c) + v_c;
% f4 =  0.5*u_max*(t_e^2-t_c^2)-u_max*t_c*(t_e-t_c)+v_c*(t_e-t_c)+ p_c;
% [t_c,v_c,p_c,t_e] = solve(f1 == v_c , f2 == p_c , f3 == v_e , f4 == p_e ,t_c,v_c,p_c,t_e);
syms u_min u_max v_min t_s t1 t2 tf pStart p_1 p_2 pEnd vStart v_1 v_2 vEnd 
t_s=0;
f1 = u_min*(t1-t_s) + vStart;
f2 =  0.5*u_min*(t1^2-t_s^2)-u_min*t1*(t1-t_s)+vStart*(t1-t_s)+ pStart;
f3 = p_1 + v_min*(t2-t1);
f4 = u_max*(tf-t2) + v_min;
f5 =  0.5*u_max*(tf^2-t2^2)-u_max*t2*(tf-t2)+v_min*(tf-t2)+ p_2;
[t1,t2,tf,p_1,p_2] = solve(f1 == v_min , f2 == p_1 ,f3 == p_2, f4 == vEnd , f5 == pEnd ,t1,t2,tf,p_1,p_2)




