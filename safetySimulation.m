clear all;
clc;
close all;
%% Global
global tStartSim; global tEndSim;
global length; global gamma; global phi;
global a_k;global b_k;global c_k;global d_k;
global t0
global timeStep
global tolerance
%% Simulation Settings
tolerance = 0.01;
length = 300;
constrained = false;
PLOT = true;
timeStep = 0.001;
%Dynamic Constraints
u_min = -1;
u_max = 1;
v_max = 35;
v_min = 5;
%Scheduling Safety
timeHeadway = 1;
%Rear-End Safety
gamma = 10; %Standstill distance
phi = 0.3; %Reaction time

%%
totalVehicles = 2;
v0 = [25 36];
vf = [20 20];
t0 = [-1.5 0];
tf = [12.5 15];
%% Check rear end Safety constraint at the beginning
0.5*u_min*timeHeadway^2+v0(1)*timeHeadway-phi*v0(2)-gamma
%% Check rear end Safety constraint at the end
0.5*u_min*timeHeadway^2+vf(1)*timeHeadway-phi*vf(2)-gamma
%% CAV1 and CAV2
for i = 1:totalVehicles
    x(i).Position=[];
    x(i).Velocity=[];
    x(i).Control=[];
    x(i).Time = [];
    x(i).Constants =[];
end
%% Simulation for unconstrained
if ~constrained
    
    tStartSim = min(t0);
    tEndSim = max(tf);
    
    for time = tStartSim:timeStep:tEndSim
        for  i = 1:totalVehicles
            if time < t0(i)
                %CAV has not been entered yet.
                continue
            end
            [a,b,c,d] = energyMatrix(t0(i),tf(i),0,length,v0(i),vf(i));
            x(i).Velocity(end+1) = 0.5*a*(time)^2+b*(time)+c;
            x(i).Position(end+1) = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
            x(i).Control(end+1) = a*(time)+b;
            x(i).Constants = [a,b,c,d];
            x(i).Time(end+1) = time;
        end
        
    end
    
end
%% Plot
if PLOT
    for i =1:totalVehicles
        figure(1)
        plot(x(i).Time,x(i).Position)
        hold on
        ylabel({'Position'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontSize',8,...
            'FontName','Times')
        xlabel({'Time $(s)$'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontWeight','normal',...
            'FontSize',8,...
            'FontName','Times')
        
        figure(2)
        plot(x(i).Time,x(i).Velocity)
        hold on
        ylabel({'Velocity'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontSize',8,...
            'FontName','Times')
        xlabel({'Time $(s)$'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontWeight','normal',...
            'FontSize',8,...
            'FontName','Times')
        
        figure(3)
        plot(x(i).Time,x(i).Control)
        hold on
        ylabel({'Control'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontSize',8,...
            'FontName','Times')
        xlabel({'Time $(s)$'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontWeight','normal',...
            'FontSize',8,...
            'FontName','Times')
    end
    
end
%% Find Rear-End Distance and safeDistance

indexInit = find(x(1).Time == x(2).Time(1));
indexFinal = find(x(1).Time == x(2).Time(end));


distance = x(1).Position(indexInit:indexFinal)-x(2).Position;

safeDistance = phi*x(2).Velocity + gamma;
figure(4)
plot(x(2).Time,distance)
hold on
plot(x(2).Time,safeDistance,'--r')
ylabel('Rear-end distance')
xlabel('Time')

%% Defining variables for fsolve
a_k = x(1).Constants(1);
b_k = x(1).Constants(2);
c_k = x(1).Constants(3);
d_k = x(1).Constants(4);
global v_0; global v_f; global p_0; global p_f; global t_0; global t_f

v_0 = v0(2);
v_f = vf(2);
p_0 = 0;
p_f = length;
t_0 = t0(2);
t_f = tf(2);

%%
x0 = [-2,-2]*2;
options = optimset('Display','iter','PlotFcns',@optimplotfval,'MaxIter',50000,'MaxFunEvals',10000,'TolFun',1e-13,'TolX',1e-13);
[X, err] = fminsearch(@SafetyError, x0,options);

%%
a = X(1);
b = X(2);

c = v_0;
d = p_0;

x(2).Position=[];
x(2).Velocity=[];
x(2).Control=[];
x(2).Time = [];
s =[];
p = 0;
secondArc = false;
jump = 0;
for time = tStartSim:timeStep:tEndSim
    if time < t0(2)
        %CAV has not been entered yet.
        continue
    end
    
    index = int64(time/timeStep+1);
    posK = (1/6)*a_k*(time)^3+0.5*b_k*(time)^2+c_k*(time)+d_k;
    
    if jump == 0
        x(2).Velocity(end+1) = 0.5*a*(time)^2+b*(time)+c;
        x(2).Position(end+1) = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
        x(2).Control(end+1) = a*(time)+b;
    elseif  jump == 1
        x(2).Velocity(end+1) =  0.5*e*(time)^2+f*(time)+g;
        x(2).Position(end+1) = (1/6)*e*(time)^3+0.5*f*(time)^2+g*(time)+h;
        x(2).Control(end+1) = e*(time)+f;
    end
    
    x(2).Time(end+1) = time;
    distance = posK-x(2).Position(end);
    safeDistance = phi*x(2).Velocity(end) + gamma;
    s(end+1) = distance - safeDistance;
    
    if jump==0 && distance-safeDistance <= -tolerance
        t1=time;
        [e,f,g,h] = energyMatrix(t1,t_f,x(2).Position(end),p_f,x(2).Velocity(end),v_f);
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


error1 =(p_f - x(2).Position(end))^2;
error2 =(v_f - x(2).Velocity(end))^2;
%Hamiltonian should be constant

if jump ==1
    if IndexActive > (tEndSim/timeStep)
        error3 = 1000;
        error5 =10000;
    else
        error5 = ((a*c-b^2/2)-(e*g-f^2/2));
        error3 = (x(2).Velocity(IndexActive)-x(2).Velocity(IndexActive+1))^2;%Continuity in speed
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
error4 = trapz(x(2).Time,s)^2;
% error1
% error2
% error3
% error4

error =error1+error2+error3+error4+error5;
fprintf('Error in final Position with %4.6f \n',sqrt(error1));
fprintf('Error in final Speed with %4.6f \n',sqrt(error2));
fprintf('Error in Intemediate Speed %4.6f \n',sqrt(error3));
fprintf('Error in Violation %4.6f \n',sqrt(error4));
fprintf('Error in Hamiltonian %4.6f \n',error5);
fprintf('Error in  P for lambdas %4.6f \n',sqrt(error6));


fprintf('++++++++++++++ \n');



%%

% %% Using fsolve
% fun = @(x)RearEnd(x,a_k, b_k,c_k,d_k, t_0, t_f, p_0, p_f, v_0, v_f, phi,gamma);
% a0  = x(2).Constants(1);
% b0  = x(2).Constants(2);
% c0  = x(2).Constants(3);
% d0  = x(2).Constants(4);
% t10 = x(2).Time(indexActivate);
% options=optimset('PlotFcns',{@optimplotx,@optimplotfval},'disp','iter','LargeScale','ON','TolFun',0.00001);
% x0 =[a_k/2,b_k,24.9696,d_k,a_k,b_k,c_k,d_k,5,4];
% [sol,fval,exitflag,output]  = fsolve(fun,x0,options)
% % if exitflag~=1
% %     msg = 'solver crashed';
% %     error(msg)
% % end
% %% Reseting CAV 2
% x(2).Position=[];
% x(2).Velocity=[];
% x(2).Control=[];
% x(2).Time = [];
% %% Finding trajectory for CAV 2
% a = sol(1);b = sol(2);c = sol(3);d = sol(4);g = sol(5);h = sol(6);m = sol(7);n = sol(8);P = sol(9);t1 = sol(10);
% %%
% p1 = (1/6)*a*(t1)^3+0.5*b*(t1)^2+c*(t1)+d
% v1 = 0.5*a*(t1)^2+b*(t1)+c
% u1 =a*(t1)+b
% pk = (1/6)*a_k*(t1)^3+0.5*b_k*(t1)^2+c_k*(t1)+d_k
% vk = 0.5*a_k*(t1)^2+b_k*(t1)+c_k
% uk =a_k*(t1)+b_k
% p1 - pk + phi*v1
% u1^2/(2*(v1+phi*u1))
% t1
% omega = P*(v1+phi*u1)+0.5*u1^2
% %%
% i = 2;
% for time = tStartSim:timeStep:tEndSim
%     if time < t0(i)
%         %CAV has not been entered yet.
%         continue
%     end
%     if time < t1 %First Arc
%         x(i).Velocity(end+1) = 0.5*a*(time)^2+b*(time)+c;
%         x(i).Position(end+1) = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
%         x(i).Control(end+1) = a*(time)+b;
%     else %Second arc
%         x(i).Velocity(end+1) = 0.5*g*(time)^2+h*(time)+m;
%         x(i).Position(end+1) = (1/6)*g*(time)^3+0.5*h*(time)^2+m*(time)+n;
%         x(i).Control(end+1) = g*(time)+h;
%     end
%     x(i).Time(end+1) = time;
% end
%% Plot
if PLOT
    for i =1:totalVehicles
        figure(6)
        plot(x(i).Time,x(i).Position)
        hold on
        ylabel({'Position'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontSize',8,...
            'FontName','Times')
        xlabel({'Time $(s)$'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontWeight','normal',...
            'FontSize',8,...
            'FontName','Times')
        
        figure(7)
        plot(x(i).Time,x(i).Velocity)
        hold on
        ylabel({'Velocity'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontSize',8,...
            'FontName','Times')
        xlabel({'Time $(s)$'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontWeight','normal',...
            'FontSize',8,...
            'FontName','Times')
        
        figure(8)
        plot(x(i).Time,x(i).Control)
        hold on
        ylabel({'Control'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontSize',8,...
            'FontName','Times')
        xlabel({'Time $(s)$'},...
            'FontUnits','points',...
            'interpreter','latex',...
            'FontWeight','normal',...
            'FontSize',8,...
            'FontName','Times')
    end
    
end
%% Find Rear-End Distance and safeDistance
plot(x(i).Time,x(i).Control)
indexInit = find(x(1).Time == x(2).Time(1));
indexFinal = find(x(1).Time == x(2).Time(end));

distanceCons = x(1).Position(indexInit:indexFinal)-x(2).Position;

safeDistanceCons = phi*x(2).Velocity + gamma;
figure(9)
plot(x(2).Time,distanceCons,'-b')
hold on
plot(x(2).Time,safeDistanceCons,'--g')
ylabel('Rear-end distance')
xlabel('Time')