function [tf,tIntermediate,constrained,t1,t2] = deadline(vStart,vEnd,pStart,pEnd)
%With giving initial state(position and velocity) and final state it will
%find the process time (Scheduling Terminology-> shortest time that a job
%can be done, with considering control input constraints) if constrained
%becomes active, t1 is entrance to the constrained arc and t2 is exit from
%it.

global u_max
global u_min
global v_min

pIntermediate = (vEnd^2-vStart^2+2*(u_min*pStart-u_max*pEnd))/(-2*(u_max-u_min));
CHECK = vStart^2+2*u_min*(pIntermediate - pStart);



if CHECK > 0
    vIntermediate = sqrt(vStart^2+2*u_min*(pIntermediate - pStart));
    tIntermediate = (vIntermediate - vStart)/u_min;
    tf = tIntermediate + (vEnd - vIntermediate)/u_max;
    constrained = false;
    t1 = 0;
    t2 = 0;
else  %in this case thye constraint for speed= vmax became active
    
    constrained = true;
    tIntermediate = 0;%This is a dummy variable in this case.
    
    t1 = -(vStart - v_min)/u_min;

    tf = -(u_min*vEnd^2 - 3*u_max*vStart^2 + u_min*v_min^2 - 3*u_max*v_min^2 - 2*pEnd*u_min*u_max + 2*pStart*u_min*u_max - 2*u_min*vEnd*v_min + 6*u_max*vStart*v_min)/(2*u_min*u_max*v_min);

    t2 = (3*u_max*vStart^2 - u_min*vEnd^2 + u_min*v_min^2 + 3*u_max*v_min^2 + 2*pEnd*u_min*u_max - 2*pStart*u_min*u_max - 6*u_max*vStart*v_min)/(2*u_min*u_max*v_min);

end
end

