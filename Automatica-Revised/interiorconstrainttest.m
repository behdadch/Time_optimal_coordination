clear
clc
L = 100;%300;
D = 50;
S = 30;
%T = [13.27,25.38,26.12,26.86,38.87];
%p = [0,L,L+0.5*S,L+S,2*L+S];
%v = [25,20,20,20,25];
v =10;
T = [0,8,15];%[0,12,16,23,26];
p = [0,L,L+S];
%v = [25,20,20,20,25];
%X = energyMatrixInterior(T,p,v);
X = energyMatrixInteriorWOSpeed(T,p,v); % when we dont set speed constraint 
vel = [];
pos =[];
control =[];
lambdaP = [];
lambdaV = [];
time = [];
%%
for t=0:0.01:max(T)
    if t<T(1)
        continue
    elseif t<=T(2)
        C = X(1:4);
    elseif t<=T(3)
        C = X(5:8);
  %  elseif t<=T(4)
   %     C = X(9:12);
   % elseif t<=T(5)
   %     C = X(13:16);
    end
        vel(end+1) = 0.5*C(1)*(t)^2+C(2)*(t)+C(3);
        pos(end+1) = (1/6)*C(1)*(t)^3+0.5*C(2)*(t)^2+C(3)*(t)+C(4);
        control(end+1) = C(1)*(t)+C(2);
        lambdaV(end+1) = - control(end);
        lambdaP(end+1) = C(1);
        time(end+1) = t;
        %pJump  = X(end:19);
        pJump  = X(end);
end
%%
figure
subplot(3,1,1)
plot(time,pos,'-k','LineWidth',1.2)
hold on
ylim = get(gca,'ylim');
for i=1:length(T)
    plot([T(i),T(i)],ylim,'--r')
    
end 
grid on
xlabel('Time')
ylabel('Position')

subplot(3,1,2)
plot(time,vel,'-k','LineWidth',1.2)
hold on
ylim = get(gca,'ylim');
for i=1:length(T)
    plot([T(i),T(i)],ylim,'--r')
    
end 
grid on
xlabel('Time')
ylabel('speed')

subplot(3,1,3)
plot(time,control,'-k','LineWidth',1.2)

hold on
ylim = get(gca,'ylim');
for i=1:length(T)
    plot([T(i),T(i)],ylim,'--r')
    
end 
grid on
xlabel('Time')
ylabel('Control')
%%
hamiltonian = 0.5*control.^2 + lambdaP .* vel + lambdaV .* control;

figure
plot(time,hamiltonian,'-k','LineWidth',1.2)
ylabel('Hamiltonian')

figure
plot(time,lambdaP,'-k','LineWidth',1.2)
hold on


%%First Jump 
%%
for i=1
lambdap1n = interp1(time,lambdaP,T(i+1)-0.01);
lambdap1p = interp1(time,lambdaP,T(i+1)+0.01);
hamilton1n = interp1(time,hamiltonian,T(i+1)-0.01);
hamilton1p = interp1(time,hamiltonian,T(i+1)+0.01);
speedatJump = interp1(time,vel,T(i+1));
hamilton1nAd = interp1(time,hamiltonian,T(i+1)+0.01)-pJump;

hamilton1n - hamilton1p
end



%%

%% fuel consumption ml/s
% f1 = fuel(vel,control);
% f2 = f1(control>=0);
% averageRate = mean(f2);
% Cons = (averageRate*range(T))/1000;
% fprintf('Average fuel Rate :%4.2f ml/s \t total fuel/liter: %4.7f   \n',averageRate,Cons);

