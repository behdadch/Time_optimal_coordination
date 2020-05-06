function error = SafetyError(x)
a = x(1);
b = x(2);
global tStartSim tEndSim;
global gamma phi;
global a_k b_k c_k d_k;
global v_0  v_f  p_0  p_f;
global t0;
global t_f
global timeStep
global tolerance
c = v_0;
d = p_0;

position=[];
velocity=[];
control=[];
s = [];
t =[];

jump = 0;
secondArc = false;
for time = tStartSim:timeStep:tEndSim
    if time < t0(2)
        %CAV has not been entered yet.
        continue
    end
    t(end+1)  = time;
    posK = (1/6)*a_k*(time)^3+0.5*b_k*(time)^2+c_k*(time)+d_k;
    index = int64(time/timeStep+1);
    if jump == 0
        velocity(end+1) = 0.5*a*(time)^2+b*(time)+c;
        position(end+1) = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
        control(end+1) = a*(time)+b;
    elseif jump ==1
        velocity(end+1) = 0.5*e*(time)^2+f*(time)+g;
        position(end+1) = (1/6)*e*(time)^3+0.5*f*(time)^2+g*(time)+h;
        control(end+1) = e*(time)+f;
    else
        msg='WTF';
        error(msg);
    end
    
    distance = posK-position(end);
    safeDistance = phi*velocity(end) + gamma;
    s(end+1) = distance - safeDistance;
    if jump==0 && distance-safeDistance <= -tolerance
        t1=time;
        [e,f,g,h] = energyMatrix(t1,t_f,position(end),p_f,velocity(end),v_f);
        %         p = -0.5*control(end)^2/(velocity(end)+phi*control(end));%p(t1)
        %         t1 = time;
        %         e = a - p;
        %         f = a*time+b-e*time+phi*p;
        %         %m = velocity(end)-0.5*g*(time)^2+h*(time);
        %         g = (a*c-0.5*b^2+0.5*f^2)/e;
        %         h = position(end)-((1/6)*e*(time)^3+0.5*f*(time)^2+g*(time));
        IndexActive = index;
        jump = 1;
    end
end


error1 =(p_f - position(end))^2;
error2 =(v_f - velocity(end))^2;
%Hamiltonian should be constant

if jump ==1
    if IndexActive > (tEndSim/timeStep)
        error3 = 1000000000000000000000000000000000000000000000000000000;
        error5 =10000000000000000000000000000000000000000000000000000000;
    else
        error5 = ((a*c-b^2/2)-(e*g-f^2/2))^2;
        error3 = (velocity(IndexActive)-velocity(IndexActive+1))^2;%Continuity in speed
        
    end
    lambdav1= -(a*t1+b);
    lambdav2 = -(e*t1+f);
    p1 = (a-e);
    p2 = (lambdav1 -lambdav2)/phi;
    error6 = (p1 -p2)^2;%jump conditions
else
    error3=0;
    error5=0;
    error6=0;
end


s(s>0)=0;
error4 = trapz(t,s)^2;
% error1
% error2
% error3
% error4
sqrt(error6)
error =10000*error1+1000*error2+error3+10000000*error4+20*error5+1000*error6;
end

