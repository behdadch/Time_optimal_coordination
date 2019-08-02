function [pos,vel,control,solved]=controller(i,j,type,pathInfo,time,path,solved)
global T
global u_min
global u_max
global v_max
global v_min
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
elseif type(i,j) == "Latest-Time"
    [t,tc,constrained,t1,t2]= deadline(vStart,vEnd,pStart,pEnd);
    if constrained == false
        if T(i,j)<= time && time <= T(i,j) + tc
            control = u_min;
            vel = control*(time-T(i,j))+vStart;
            pos = 0.5*control*(time^2-T(i,j)^2)-control*T(i,j)*(time-T(i,j))+vStart*(time-T(i,j))+pStart;
        else
            control = u_max;
            TX = T(i,j) + tc;
            PC = 0.5*u_min*(TX^2-T(i,j)^2)-u_min*T(i,j)*(TX-T(i,j))+vStart*(TX-T(i,j))+pStart;
            VC = u_min*(TX-T(i,j))+vStart;
            vel = control*(time-TX) + VC;
            pos = 0.5*control*(time^2-TX^2)-control*TX*(time-TX)+VC*(time-TX)+PC;
        end
    elseif constrained == true
        if T(i,j)<= time && time <= T(i,j) + t1
            control = u_min;
            vel = control*(time-T(i,j))+ vStart;
            pos = 0.5*control*(time^2-T(i,j)^2)-control*T(i,j)*(time-T(i,j))+vStart*(time-T(i,j))+pStart;
        elseif T(i,j)+ t1 <= time && time <= T(i,j) + t2
            control = 0;
            vel = v_min;
            TX = T(i,j)+ t1;
            Pt1 = 0.5*u_min*(TX^2-T(i,j)^2)-u_min*T(i,j)*(TX-T(i,j))+vStart*(TX-T(i,j))+pStart;
            pos = vel*(time-TX)+Pt1;
        elseif T(i,j)+ t2 <= time && time <= T(i,j) + t
            control = u_max;
            TX = T(i,j)+ t1;
            TX2 = T(i,j)+ t2;
            Pt1 = 0.5*u_min*(TX^2-T(i,j)^2)-u_min*T(i,j)*(TX-T(i,j))+vStart*(TX-T(i,j))+pStart;
            Pt2 = v_min*(TX2-TX)+Pt1;
            vel = control*(time-TX2)+v_min;
            pos = 0.5*control*(time^2-TX2^2)-control*TX2*(time-TX2)+v_min*(time-TX2)+Pt2;
        end
    else
        msg = 'Error occurred in defining "constrained" variable.';
        error(msg);
    end
elseif type(i,j) == "Umin"
    
    if solved == 0
        const = [T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_min];
        fun = @(x)MinDecActive(x,T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_min);
        x0 = zeros(1,7);
        sol = fsolve(fun,x0);
        solved = sol(1:end);
        if sol(7)<0
          x0 = zeros(1,7)+25;
          sol = fsolve(fun,x0);
          solved = sol(1:end);
        end 
    else
        sol = solved;
    end
    a = sol(1);
    b = sol(2);
    c = sol(3);
    d = sol(4);
    f = sol(5);
    g = sol(6);
    t1= sol(7);
   
%     [a,b,c,d,f,g,t1]=sol
    if T(i,j)<= time && time <= T(i,j) + t1
        vel = 0.5*a*(time)^2+b*(time)+c;       
        pos = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;        
        control = a*(time)+b;
    else
        control = u_min;
        vel = control*time + f;
        pos = 0.5*control*time^2 + f*time + g;
    end   
end