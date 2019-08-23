function [pos,vel,control,zoneNumber]=controller(i,j,type,pathInfo,x,time)
%%Author: Behdad Chalaki
%Advisor: Andreas Malikopoulos
%Phd Student at University of Delaware
%Information and decision Science Lab
%For more information, send an eamil to bchalaki@udel.edu
global T
global vMerge
global u_min
global u_max
m = pathInfo(i,j);
zoneNumber = m;
[pStart,pEnd,vStart,vEnd] = mapGeometry(i,m,pathInfo);

if type(i,j) == "Time-optimal"
    [t,tc,optimal]= timeOptimal(vStart,vEnd,pStart,pEnd,m);
    if optimal ==0
        vel = vMerge ;
        
        const = pStart - vMerge*T(i,j);% this is for the initial condition P(T(i,j))=pStart
        pos = vMerge*(time) + const;
        
        control = 0;
    else
        if T(i,j)<= time && time < T(i,j) + tc
            control = u_max;
            [b,c]= timeMatrix(pStart,pEnd,vStart,vEnd,T(i,j),T(i,j)+t,1);
        else
            control = u_min;
            [b,c]= timeMatrix(pStart,pEnd,vStart,vEnd,T(i,j),T(i,j)+t,0);
        end

        vel = control*time + b;
        
        pos = 0.5*control*time^2 + b*time + c;
        
        
    end
elseif type(i,j) == "Energy-optimal"
   %Update each time %[a,b,c,d] = energyMatrix(time,T(i,j+1),x(i).Position(end),pEnd,x(i).Velocity(end),vEnd);
    [a,b,c,d] = energyMatrix(T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd);
    vel = 0.5*a*(time)^2+b*(time)+c;
    
    pos = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
    
    control = a*(time)+b;
elseif type(i,j) == "Merging-Zone"
            vel = vMerge ;
        
        const = pStart - vMerge*T(i,j);% this is for the initial condition P(T(i,j))=pStart
        pos = vMerge*(time) + const;
        
        control = 0;
end