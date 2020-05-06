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
    %  Unconstrained--> u = Umin
    % %     if solved == 0
    % %         %const = [T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_min];
    % %         options=optimset('disp','iter','LargeScale','on','TolFun',0.001);
    % %         fun = @(x)MinDecActive(x,T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_min);
    % %
    % %         f = vEnd-u_min*T(i,j+1);
    % %         g = pEnd-0.5*u_min*T(i,j+1)^2-f*T(i,j+1);
    % %
    % %         x0 = [a,b,c,d,e,f,g,t1];%initial guess
    % %
    % %         [sol,fval,exitflag,output]  = fsolve(fun,x0,options)
    % %         solved = sol(1:end);
    % %         if sol(7)>T(i,j+1) || exitflag <=0
    % %             msg = 'Error occurred.';
    % %             error(msg)
    % %         end
    % %     else
    % %         sol = solved;
    % %     end
    % %     a = sol(1);
    % %     b = sol(2);
    % %     c = sol(3);
    % %     d = sol(4);
    % %     f = sol(5);
    % %     g = sol(6);
    % %     t1= sol(7);
    % %
    % %
    [a,b,c,d,f,g,t1] = UlimMatrix(T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_min);
    solved.constants = [a,b,c,d,f,g,t1];
    if T(i,j)<= time && time <= t1
        vel = 0.5*a*(time)^2+b*(time)+c;
        pos = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
        control = a*(time)+b;
    else
        control = u_min;
        vel = control*time + f;
        pos = 0.5*control*time^2 + f*time + g;
    end
elseif type(i,j) == "Umax"
    if solved.done == false
        %const = [T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_min];
        options=optimset('disp','iter','LargeScale','on','TolFun',0.001);
        fun = @(x)ControlActive(x,T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_max);
        x0 = zeros(1,7)+25
        [sol,fval,exitflag,output]  = fsolve(fun,x0,options)
        solved.constants = sol(1:end);
        if  exitflag <=0
            msg = 'Error occurred.';
            error(msg)
        end
    else
        sol = solved.constants;
    end
    a = sol(1);
    b = sol(2);
    c = sol(3);
    d = sol(4);
    f = sol(5);
    g = sol(6);
    t1= sol(7);
    
    %  Unconstrained--> u = Umax
    
    if T(i,j)<= time && time <= t1
        control = u_max;
        vel = control*time + f;
        pos = 0.5*control*time^2 + f*time + g;
        
    else
        vel = 0.5*a*(time)^2+b*(time)+c;
        pos = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
        control = a*(time)+b;
    end
elseif type(i,j) == "UmaxUnconsUmin"
    %const = [T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_min];
    if solved.done == false
        options=optimset('disp','iter','LargeScale','on','TolFun',0.001);
        fun = @(x)UmaxUnconUmin(x,T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_max,u_min);
        
        x02= solved.constants; %[a,b,c,d,f,g,t1]
        a0  =solved.constants(1);
        b0  = solved.constants(2);
        c0  = solved.constants(3);
        d0  = solved.constants(4);
        g0  = solved.constants(5);
        h0  = solved.constants(6);
        t20 = solved.constants(7);
        
        t10= (u_max-b0)/a0;
        e0 = vStart - u_max*T(i,j);
        f0 = -0.5*u_max*T(i,j)^2-T(i,j)*e0+pStart;
        
        x0 = [e0,f0,t10,a0,b0,c0,d0,t20,g0,h0];
        [sol,fval,exitflag,output]  = fsolve(fun,x0,options)
        solved.constants = sol(1:end);
        if exitflag <=0
            msg = 'Error occurred.';
            error(msg)
        end
        solved.done = true;
    else
        sol = solved.constants;
        
    end
    
    e  = sol(1);
    f  = sol(2);
    t1 = sol(3);
    a  = sol(4);
    b  = sol(5);
    c  = sol(6);
    d  = sol(7);
    t2 = sol(8);
    g  = sol(9);
    h  = sol(10);
    
    %  u = Umax--> Unconstrained--> u = Umin
    
    if T(i,j)<= time && time <= t1
        control = u_max;
        vel = control*time + e;
        pos = 0.5*control*time^2 + e*time + f;
        
    elseif t1<= time && time <= t2
        vel = 0.5*a*(time)^2+b*(time)+c;
        pos = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
        control = a*(time)+b;
    else
        control = u_min;
        vel = control*time + g;
        pos = 0.5*control*time^2 + g*time + h;
    end
    
end