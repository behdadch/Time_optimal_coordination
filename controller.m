function [pos,vel,control]=controller(i,j,type,pathInfo,time,path)
global T
global u_min
global u_max
global v_max
PathNumber = path(i);
m = pathInfo(PathNumber,j);
[pStart,pEnd,vStart,vEnd] = mapGeometry(i,m,pathInfo,path);


if type(i,j) == "Time-optimal"
    [t,tc,constrained,t1,t2]= timeOptimal(vStart,vEnd,pStart,pEnd);
    if constrained == false
        if T(i,j)<= time && time <= T(i,j) + tc
            control = u_max;
            [b,c]= timeMatrix(pStart,pEnd,vStart,vEnd,T(i,j),T(i,j)+t,1);
        else
            control = u_min;
            [b,c]= timeMatrix(pStart,pEnd,vStart,vEnd,T(i,j),T(i,j)+t,0);
        end
        
        vel = control*time + b;
        
        pos = 0.5*control*time^2 + b*time + c;
    elseif constrained == true
        if T(i,j)<= time && time <= T(i,j) + t1
            control = u_max;
            vel = control*(time-T(i,j))+ vStart;
            pos = 0.5*control*(time^2-T(i,j)^2)-control*T(i,j)*(time-T(i,j))+vStart*(time-T(i,j))+pStart;
        elseif T(i,j)+ t1 <= time && time <= T(i,j) + t2
            control = 0;
            vel = v_max;
            TX = T(i,j)+ t1;
            Pt1 = 0.5*u_max*(TX^2-T(i,j)^2)-u_max*T(i,j)*(TX-T(i,j))+vStart*(TX-T(i,j))+pStart;
            pos = vel*(time-TX)+Pt1; 
        elseif T(i,j)+ t2 <= time && time <= T(i,j) + t
            control = u_min;
            TX = T(i,j)+ t1;
            TX2 = T(i,j)+ t2;
            Pt1 = 0.5*u_max*(TX^2-T(i,j)^2)-u_max*T(i,j)*(TX-T(i,j))+vStart*(TX-T(i,j))+pStart;
            Pt2 = v_max*(TX2-TX)+Pt1; 
            vel = control*(time-TX2)+v_max;
            pos = 0.5*control*(time^2-TX2^2)-control*TX2*(time-TX2)+v_max*(time-TX2)+Pt2;
        end 
    else
        msg = 'Error occurred in defining "constrained" variable.';
        error(msg);        
    end
    
elseif type(i,j) == "Energy-optimal"
    %Update each time %[a,b,c,d] = energyMatrix(time,T(i,j+1),x(i).Position(end),pEnd,x(i).Velocity(end),vEnd);
    [a,b,c,d] = energyMatrix(T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd);
    vel = 0.5*a*(time)^2+b*(time)+c;
    
    pos = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
    
    control = a*(time)+b;
    % elseif type(i,j) == "Merging-Zone"
    %             vel = vMerge ;
    %
    %         const = pStart - vMerge*T(i,j);% this is for the initial condition P(T(i,j))=pStart
    %         pos = vMerge*(time) + const;
    %
    %         control = 0;
elseif type(i,j) == "Umin"
    t1
    t2
    
    
end