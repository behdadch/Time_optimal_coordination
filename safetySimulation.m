clear all;
clc;
close all;

%% Simulation Settings

length = 300;
constrained = false;
PLOT = true;
timeStep = 0.01;
%Dynamic Constraints
u_min = -2;
u_max = 1;
v_max = 35;
v_min = 5;
%Scheduling Safety
timeHeadway = 0.5;
%Rear-End Safety
gamma = 3; %Standstill distance
phi = 0.2; %Reaction time

%%
totalVehicles = 2;
v0 = [25 30];
vf = [20 20];
t0 = [0 0.5];
tf = [13 14.1];
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
indexActivate = find(-safeDistance +distance <0.1 ,1);
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
v_0 = v0(2);
v_f = vf(2);
p_0 = 0;
p_f = length;
t_0 = t0(2);
t_f = tf(2);


%% Using fsolve
fun = @(x)RearEnd(x,a_k, b_k,c_k,d_k, t_0, t_f, p_0, p_f, v_0, v_f, phi,gamma);
a0  = x(2).Constants(1);
b0  = x(2).Constants(2);
c0  = x(2).Constants(3);
d0  = x(2).Constants(4);
t10 = x(2).Time(indexActivate);
options=optimset('disp','iter','LargeScale','on','TolFun',0.0001);
x0 = [a0,b0,c0,d0,a_k,b_k,c_k,d_k,12,t10];
[sol,fval,exitflag,output]  = fsolve(fun,x0,options)
% if exitflag~=1
%     msg = 'solver crashed';
%     error(msg)
% end
%% Reseting CAV 2
x(2).Position=[];
x(2).Velocity=[];
x(2).Control=[];
x(2).Time = [];
x(2).Constants =[];
%% Finding trajectory for CAV 2
x(2).Constants = sol;
a = sol(1);b = sol(2);c = sol(3);d = sol(4);g = sol(5);h = sol(6);m = sol(7);n = sol(8);P = sol(9);t1 = sol(10);

i = 2;
for time = tStartSim:timeStep:tEndSim
    if time < t0(i)
        %CAV has not been entered yet.
        continue
    end
    if time < t1 %First Arc
        x(i).Velocity(end+1) = 0.5*a*(time)^2+b*(time)+c;
        x(i).Position(end+1) = (1/6)*a*(time)^3+0.5*b*(time)^2+c*(time)+d;
        x(i).Control(end+1) = a*(time)+b;
    else %Second arc
        x(i).Velocity(end+1) = 0.5*g*(time)^2+h*(time)+m;
        x(i).Position(end+1) = (1/6)*g*(time)^3+0.5*h*(time)^2+m*(time)+n;
        x(i).Control(end+1) = g*(time)+h;
    end
    x(i).Time(end+1) = time;
end
%% Plot
if PLOT
    for i =1:totalVehicles
        figure(5)
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
        
        figure(6)
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
        
        figure(7)
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


distanceCons = x(1).Position(indexInit:indexFinal)-x(2).Position;

safeDistanceCons = phi*x(2).Velocity + gamma;
figure(8)
plot(x(2).Time,distanceCons,'-b')
hold on
plot(x(2).Time,safeDistanceCons,'--g')
ylabel('Rear-end distance')
xlabel('Time')