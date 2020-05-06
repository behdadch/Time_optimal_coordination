function [tf,tIntermediate,constrained,t1,t2] = timeOptimal(vStart,vEnd,pStart,pEnd)
%With giving initial state(position and velocity) and final state it will
%find the process time (Scheduling Terminology-> shortest time that a job
%can be done, with considering control input constraints)
%if constrained becomes active, t1 is entrance to the constrained arc and
%t2 is exit from it. 

global u_max
global u_min
global v_max

pIntermediate = (vEnd^2-vStart^2+2*(u_max*pStart-u_min*pEnd))/(2*(u_max-u_min));
vIntermediate = sqrt(vStart^2+2*u_max*(pIntermediate - pStart));

if vIntermediate < v_max
tIntermediate = (vIntermediate - vStart)/u_max;
tf = tIntermediate + (vEnd - vIntermediate)/u_min;
constrained = false;
t1 = 0;
t2 = 0;
else  %in this case thye constraint for speed= vmax became active
constrained = true;
tIntermediate = 0;%This is a dummy variable in this case.
t1 = (v_max-vStart)/u_max;
tf  = -(u_max*vEnd^2 - u_min*vStart^2 - u_min*v_max^2 + u_max*v_max^2 - 2*pEnd*u_min*u_max + 2*pStart*u_min*u_max - 2*u_max*vEnd*v_max + 2*u_min*vStart*v_max)/(2*u_min*u_max*v_max);
t2 = tf + (v_max-vEnd)/u_min;
end 

end

