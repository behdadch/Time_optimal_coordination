function [t,tIntermediate,optimal] = timeOptimal(vStart,vEnd,pStart,pEnd,zone)
%With giving initial state(position and velocity) and final state it will
%find the process time (Scheduling Terminology-> shortest time that a job
%can be done, with considering control input constraints)
%If the merging is 1, it means that we are in the merging zone and velocity
%is constant
global u_max
global u_min
merging = 0 ; 
switch merging
    case 0
        pIntermediate = (vEnd^2-vStart^2+2*(u_max*pStart-u_min*pEnd))/(2*(u_max-u_min));
        vIntermediate = sqrt(vStart^2+2*u_max*(pIntermediate - pStart));
        tIntermediate = (vIntermediate - vStart)/u_max;
        t = tIntermediate + (vEnd - vIntermediate)/u_min;
        optimal = 1; 
    case 1       
        t = (pEnd - pStart)/(vStart);
        tIntermediate = 0;
        optimal = 0;
end

end

