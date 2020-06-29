function [pos,vel,control,solved]=controller(i,j,type,pathInfo,time,path,solved,z)
global T
global u_min
global u_max
global v_max
global v_min
PathNumber = path(i);
m = pathInfo(PathNumber,j);
[pStart,pEnd,vStart,vEnd] = mapGeometry(i,m,pathInfo,path,z);


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
    solved.constants = [a,b,c,d,0,0,0];
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
    [a,b,c,d,f,g,t1] = UlimMatrix(T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_min);
    
    
    if t1<T(i,j+1)
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
        %fprintf('CAV %d enters U_min at %4.2f ,[%4.2f,%4.2f]\n',i,t1,T(i,j),T(i,j+1))
        
    else
        %%u_min --> unconstrained
        if solved.done == false || solved.constants(7)==0
            options=optimset('MaxIter',50000,'MaxFunEvals',10000,'TolFun',1e-13,'TolX',1e-13,'Display', 'off');
            fun = @(x)ControlActive(x,T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_min);
            x0 = zeros(1,7);
            [sol,fval,exitflag,output]  = fsolve(fun,x0,options);
            solved.constants = sol(1:end);
            solved.done = true;
            
            count = 0;
            while(exitflag<=0 || sol(7)<T(i,j))
                [a,b,c,d] = energyMatrix(T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd);
                x0 = [a,b,c,d,0,0,T(i,j)]+0.1*count;
                [sol,fval,exitflag,output]  = fsolve(fun,x0,options);
                solved.constants = sol(1:end);
                solved.done = true;
                count = count +1
            end

            if  exitflag <=0
                fprintf('%s for CAV %d.\n', output.message,i);
                msg = 'Error occurred. Maybe change the x0';
                x0
                fval
                sol
                error(msg)
            end
            fprintf('CAV %d exit U_min to unconst at %4.2f ,[%4.2f,%4.2f]\n',i,sol(7),T(i,j),T(i,j+1));
        else
            %             if(i==12)
            %                 fprintf('CAV %d exit U_min to unconst at %4.2f ,[%4.2f,%4.2f]\n',i,solved.constants(7),T(i,j),T(i,j+1));
            %             end
            sol = solved.constants;
        end
        a = sol(1);
        b = sol(2);
        c = sol(3);
        d = sol(4);
        f = sol(5);
        g = sol(6);
        t1= sol(7);
        
        if T(i,j)<= time && time <= t1
            control = u_min;
            vel = control*time + f;
            pos = 0.5*control*time^2 + f*time + g;
            
        else
            vel = 0.5*a*(time)^2+b*(time)+c;
            pos = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
            control = a*(time)+b;
        end
    end
    
    
elseif type(i,j) == "Umax"
    if solved.done == false || numel(solved.constants)~=7 || solved.constants(7)==0
        options=optimset('MaxIter',500000,'MaxFunEvals',100000,'TolFun',1e-13,'TolX',1e-13,'Display', 'off');
        fun = @(x)ControlActive(x,T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_max);
        x0 = solved.constants+5;
        if numel(solved.constants)~=7
            [a,b,c,d] = energyMatrix(T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd);
            x0 = [a,b,c,d,0,0,T(i,j)]+2;
        end
        [sol,fval,exitflag,output]  = fsolve(fun,x0,options);
        solved.constants = sol(1:end);
        solved.done = true;
        count = 0;
        while(exitflag<=0 || sol(7)<T(i,j))
            [a,b,c,d] = energyMatrix(T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd);
            x0 = [a,b,c,d,0,0,T(i,j)]+0.1*count;
            [sol,fval,exitflag,output]  = fsolve(fun,x0,options);
            solved.constants = sol(1:end);
            solved.done = true;
            exitflag
            sol(7)
            count = count +1
        end
        if  exitflag <=0
            fprintf('%s for CAV %d.\n', output.message,i);
            msg = 'Error occurred.';
            x0
            fval
            sol
            time
            error(msg)
        end
        fprintf('CAV %d exit U_max to unconst at %4.2f ,[%4.2f,%4.2f]\n',i,sol(7),T(i,j),T(i,j+1))
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
    
    % u = Umax--> Unconstrained
    
    if T(i,j)<= time && time <= t1
        control = u_max;
        vel = control*time + f;
        pos = 0.5*control*time^2 + f*time + g;
        
    elseif time > t1 && time < T(i,j+1)
        vel = 0.5*a*(time)^2+b*(time)+c;
        pos = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
        control = a*(time)+b;
    end
elseif type(i,j) == "UmaxUnconsUmin"
    
    %%U_max-> uncons -> u_min
    if solved.done == false || numel(solved.constants)~=10
        options=optimset('MaxIter',50000,'MaxFunEvals',10000,'TolFun',1e-13,'TolX',1e-13,'Display','off');
        fun = @(x)UmaxUnconUmin(x,T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_max,u_min);
        x02 = solved.constants; %[a,b,c,d,f,g,t1]
        a0  = solved.constants(1);
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
        [sol,fval,exitflag,output]  = fsolve(fun,x0,options);
        solved.constants = sol(1:end);
        count = 0;
        while sol(3)>sol(8) || exitflag <=0
            x0 = zeros(1,10)+10*count;
            [sol,fval,exitflag,output]  = fsolve(fun,x0,options);
            solved.constants = sol(1:end);
            count = count+1
        end
        if exitflag <=0
            fprintf('%s for CAV %d.\n', output.message,i);
            x0
            fval
            sol
            msg = 'Error occurred.';
            error(msg)
        end
        fprintf('CAV %d exit U_max to unconst at %4.2f and enter u_min at %4.2f ,[%4.2f,%4.2f] \n',i,sol(3),sol(8),T(i,j),T(i,j+1));
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
    
elseif type(i,j) == "UminUnconsUmax"
    
    %%U_min-> uncons -> u_max
    if solved.done == false || numel(solved.constants)~=10
        options=optimset('MaxIter',50000,'MaxFunEvals',10000,'TolFun',1e-13,'TolX',1e-13,'Display','off');
        fun = @(x)UmaxUnconUmin(x,T(i,j),T(i,j+1),pStart,pEnd,vStart,vEnd,u_min,u_max);
        x02 = solved.constants; %[a,b,c,d,f,g,t1]
        a0  = solved.constants(1);
        b0  = solved.constants(2);
        c0  = solved.constants(3);
        d0  = solved.constants(4);
        g0  = solved.constants(5);
        h0  = solved.constants(6);
        t20 = solved.constants(7);
        
        t10= (u_min-b0)/a0;
        e0 = vStart - u_min*T(i,j);
        f0 = -0.5*u_min*T(i,j)^2-T(i,j)*e0+pStart;
        
        x0 = [e0,f0,t10,a0,b0,c0,d0,t20,g0,h0];
        [sol,fval,exitflag,output]  = fsolve(fun,x0,options);
        solved.constants = sol(1:end);
        count = 0;
        while sol(3)>sol(8) || exitflag <=0
            x0 = zeros(1,10)+10*count;
            [sol,fval,exitflag,output]  = fsolve(fun,x0,options);
            solved.constants = sol(1:end);
            count = count+1
        end
        if exitflag <=0
            fprintf('%s for CAV %d.\n', output.message,i);
            x0
            fval
            sol
            msg = 'Error occurred.';
            error(msg)
        end
        solved.done = true;
        fprintf('CAV %d exit U_min to unconst at %4.2f and enter u_max at %4.2f ,[%4.2f,%4.2f] \n',i,sol(3),sol(8),T(i,j),T(i,j+1));
        
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
    if t1 > t2
        fprintf('%s for CAV %d.\n', output.message,i);
        msg = 't1>t2';
        error(msg)
    end
    
    if T(i,j)<= time && time <= t1
        control = u_min;
        vel = control*time + e;
        pos = 0.5*control*time^2 + e*time + f;
        
    elseif t1<= time && time <= t2
        vel = 0.5*a*(time)^2+b*(time)+c;
        pos = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
        control = a*(time)+b;
    else
        control = u_max;
        vel = control*time + g;
        pos = 0.5*control*time^2 + g*time + h;
    end
end