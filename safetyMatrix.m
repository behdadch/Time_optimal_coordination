function [a_i, b_i, c_i, d_i, e_i, f_i, g_i, h_i, m_i, n_i, t_1]= safetyMatrix(a_k, b_k, c_k, d_k, phi, gamma, v_0, v_f, p_0, p_f, t_0, t_f)

g_i = a_k ;
h_i = b_k - a_k*phi;

a_i =a_k ;
b_i = b_k - a_k*phi;
c_i = v_f - b_k*t_f - (a_k*t_f^2)/2 + a_k*phi*t_f; %from equation 8
d_i = p_0 - t_0*v_f - (a_k*t_0^3)/6 - (b_k*t_0^2)/2 + (a_k*phi*t_0^2)/2 + (a_k*t_0*t_f^2)/2 + b_k*t_0*t_f - a_k*phi*t_0*t_f;

e_i = c_i;

m_i = v_f - b_k*t_f - (a_k*t_f^2)/2 + a_k*phi*t_f;
n_i = p_f - t_f*(v_f - b_k*t_f - (a_k*t_f^2)/2 + a_k*phi*t_f) - t_f^2*(b_k/2 - (a_k*phi)/2) - (a_k*t_f^3)/6;
f_i = d_i; 
t_1=-(- 6*a_k*phi^2*t_f - 3*a_k*phi*t_0^2 + 6*a_k*phi*t_0*t_f + 3*a_k*phi*t_f^2 + 6*b_k*phi*t_f - 6*v_f*phi + a_k*t_0^3 + 3*b_k*t_0^2 - 3*a_k*t_0*t_f^2 - 6*b_k*t_0*t_f + 6*v_f*t_0 + 6*d_k - 6*gamma - 6*p_0)/(6*a_k*phi^2 - 6*a_k*phi*t_f - 6*b_k*phi + 3*a_k*t_f^2 + 6*b_k*t_f + 6*c_k - 6*v_f);
%t_1 = -b_i/a_i;

end 