clc
clear
 HIST = true;
%%
importedDataBL = importdata("C:\Users\Behdad\Dropbox\Publications\TITS-Results\double intersection\Data-Rev\Seed42\Baseline-output-1200-15m-30s.csv");
DataBl  = importedDataBL.data;

DataCAV = load('E:\matlab_workspace\SchedulingSimulation\IEEETransactionOnIntelligent\Seed42\traj_optimal_1200_15ms_15.mat');
CAV = DataCAV.x;


%% Create struct for each Vehicle 
 num_vehicle = max(DataBl(:,2));
 if (num_vehicle ~= numel(CAV))
     fprintf('ERROR in input file')
 end 

 for i = 1:num_vehicle
     indices = find(DataBl(:,2)== i);
     vehicle(i).time = DataBl(indices,1);
     vehicle(i).velocity = DataBl(indices,3);
     vehicle(i).control = DataBl(indices,4);
 end
 %% Compute the average travel time 
 
 fprintf('==========Average Travel Time (s)==========\n');
 travel_time_BL = [];
 travel_time_CAV = [];
 
 %CAV
  for i = 1:num_vehicle
     travel_time_CAV(end+1) = range(CAV(i).Timestamp);
 end 
 average_travel_time_CAV = mean(travel_time_CAV); 
 fprintf('Average travel_time for optimal Scenario is %4.2f s.\n', average_travel_time_CAV);

 
 %Baseline

 for i = 1:num_vehicle
     travel_time_BL(end+1) = range(vehicle(i).time);
 end 
 
 max_travel_time_BL = max(travel_time_BL);
 average_travel_time_BL = mean(travel_time_BL); 
 fprintf('Average travel_time for baseline Scenario is %4.2f s.\n', average_travel_time_BL);
 travel_time_improvment = (average_travel_time_BL - average_travel_time_CAV)/average_travel_time_BL;
 fprintf('Average Travel time is reduced by %4.2f percent .\n',travel_time_improvment*100)
 
 fprintf('Max travel_time for baseline Scenario is %4.2f s.\n', max_travel_time_BL);


 %% Compute the average velocity of all CAVs
fprintf('========== Average Velocity (m/s) ==========\n');
for i = 1:num_vehicle
    mean_velocity_CAV(i) = mean(CAV(i).Velocity);
end 
average_speed_CAV = mean(mean_velocity_CAV);
fprintf('Average velocity of all CAVs for optimal Scenario is %4.2f m/s.\n', average_speed_CAV);


for i = 1:num_vehicle
    mean_velocity_BL(i) = mean(vehicle(i).velocity);
end 
average_speed_BL = mean(mean_velocity_BL);
fprintf('Average velocity of all cars for baseline Scenario is %4.2f m/s.\n', average_speed_BL);


 %% Compute the average fuel consumption in ml/s
fprintf('========== Average FuelConsumptionRate (ml/s) and Total fuel consumption (L) ==========\n');

 
 for i = 1:num_vehicle
     
    f1 = fuel(CAV(i).Velocity(2:end), CAV(i).Control);
    f2 = f1( CAV(i).Control >= 0);
   
    average_rateCAV(i) = mean(f2);
    consumption_CAV(i) = (average_rateCAV(i)*range(CAV(i).Timestamp))/1000;

end 
 average_rate_all_CAV = mean(average_rateCAV);
 average_consmption_all_CAV = mean(consumption_CAV);
 fprintf('Average fuel Rate for all CAVs for optimal scenario :%4.2f ml/s \t total fuel: %4.7f l \n',average_rate_all_CAV,average_consmption_all_CAV);

for i = 1:num_vehicle
     
    f1 = fuel(vehicle(i).velocity, vehicle(i).control);
    f2 = f1( vehicle(i).control >= 0);
   
    average_rateBL(i) = mean(f2);
    consumption_BL(i) = (average_rateBL(i)*range(vehicle(i).time))/1000;

end 
 average_rate_all_BL = mean(average_rateBL);
 average_consmption_all_BL = mean(consumption_BL);
  fprintf('Average fuel Rate for all cars for baseline scenario :%4.2f ml/s \t total fuel: %4.7f l \n',average_rate_all_BL,average_consmption_all_BL);


 % f1 = fuel(vel,control);
% f2 = f1(control>=0);
% averageRate = mean(f2);
% Cons = (averageRate*range(T))/1000;
% fprintf('Average fuel Rate :%4.2f ml/s \t total fuel/liter: %4.7f   \n',averageRate,Cons);

%% Finding v_min v_max and v_avg at each time instant 

%Baseline 
t_Min_Bl = min(DataBl(:,1));
t_Max_BL = max(DataBl(:,1));

v_min_BL = [];
v_max_BL = []; 
v_avg_BL = [];
v_min_CAV = [];
v_max_CAV = []; 
v_avg_CAV = [];
time = t_Min_Bl:0.1:t_Max_BL;
%%
for t = time    
    v_BL = [];
    v_CAV = [];
    for i=1:num_vehicle 
        v_BL(end+1) = interp1(vehicle(i).time, vehicle(i).velocity,t);
       v_CAV(end+1) = interp1(CAV(i).Timestamp, CAV(i).Velocity,t);
    end 
    
    v_min_BL(end+1) =  nanmin(v_BL);
    v_max_BL(end+1) =  nanmax(v_BL);
    v_avg_BL(end+1) =  nanmean(v_BL);
    v_min_CAV(end+1) =  nanmin(v_CAV);
    v_max_CAV(end+1) =  nanmax(v_CAV);
    v_avg_CAV(end+1) =  nanmean(v_CAV);
end

A = [time',v_min_BL',v_max_BL',v_avg_BL',v_min_CAV',v_max_CAV',v_avg_CAV'];
%% Histogram
if HIST

figure;
name = 'flow1200-meanspeedHis';
h1 = histogram(mean_velocity_BL,'Normalization','probability');
h1.FaceColor = [255,50,50]/255;
h1.BinWidth = 2;
hold on
h2 = histogram(mean_velocity_CAV,'NumBins',4,'Normalization','probability');
h2.FaceColor = [50,229,229]/255;
h2.BinWidth = 2;
yticklabels(yticks*100)
xticks(4:2:24)


xlabel('Mean Speed (m/s)')
ylabel('Percentage (%)')
legend('Baseline','Optimal')
print -dpng flow1200-meanspeedHis.png
print -depsc2 flow1200-meanspeedHis.eps


%%
figure
name = 'flow1200-TravelTimeHis';
h1 = histogram(travel_time_BL,'Normalization','probability');
h1.FaceColor = [255,50,50]/255;
h1.BinWidth = 10;

hold on
h2 = histogram(travel_time_CAV,'NumBins',4,'Normalization','probability');
h2.FaceColor = [50,229,229]/255;
h2.BinWidth = 10;
yticklabels(yticks*100)
xticks(20:10:130)

xlabel('Travel Time (s)')
ylabel('Percentage (%)')
legend('Baseline','Optimal')

print -dpng flow1200-TravelTimeHis.png
print -depsc2 flow1200-TravelTimeHis.eps

end
%
% box on
% grid on
% print -depsc2 myplot.eps