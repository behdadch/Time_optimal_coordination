%Author:Behdad Chalaki
%For more information, please refer to the paper
% Dependencies: IBM CPLEX 12.10 to solve the upper-level problem


clear
close all
%%
fprintf("+++++++++++++++++++++++\n");
global u_min
global u_max
global v_max
global v_min
global vOut
global vMerge
global conflict
global roadLength
global mergeLength
global intersectionDistance
global T
global R
global D
global timeHeadway

timeHeadway = 1.5;% safe time headway
totalZones = 22; %Number of zones in the control zone

totalPath =  30; % Number of fixed paths
totalVehicles = 24; %Number of vehicles
%%
global FIFO

PLOT = false; 
RANDOM = true; %%randomized initial time of vehicles based on poisson distribution 
CONSTRAINT = true; %consider state/control constraints
ANIMATIONPP = false;% postprocessing animation 
SAVEVIDEO = false; %Save the video 
INPUT = false; %Read data from VISSIM simulation 
CENTRALIZED = false;%upper_level schedule centralized or decentralized
FIFO = false; % upper-level schedule FIFO =true or false
UPPERLEVELTEST =false; %Only set it to true to compare different upper-levels (fifo, centralized, decentralized)
if FIFO
    fprintf("Following FIFO Structure\n")
end
%%
if INPUT
    %% This is to read initialized the car similar to VISSIM scenario 
    importedData = importdata("C:\Users\Behdad\Dropbox\Publications\TITS-Results\double intersection\Data\Seed1\Baseline-Input-400-15m-30s.csv");
    data = importedData.data;
    for i=1:length(data)
        index = find(data(:,2)==i);
        TZeros(i) = data(index,1);
        path(i) = data(index,3);
    end
    totalVehicles = length(data);
    RANDOM = false; %this is to make sure we do not use RANDOM by mistake
else
    if RANDOM
        rng(69,'twister');
        TZeros = randomTimeGen(totalVehicles);
    else
        TZeros = [0,2.0,25,4.3,5.8,7.4,9.3,10.9,11.3,11.4,12.1,13.1,13.6,14.2,14.6,16.97,26,37,42,55];
    end
    %in pathInfo(i,j)-> i is vehicle index after order calculation and the j shows the
    % zone that vehicle is in that.

    %Path information for the vehicles
    pathSequence = [1,5,3,4,6,2, 16,20,9,8,27,7, 17,21,10,24,28,13, 18,22,11,25,29,14, 19,23,12,26,30,15];
    for i = 1:totalVehicles
        indP = mod(i,totalPath);
        if indP == 0
            indP = totalPath;
        end
        path(i) = pathSequence(indP);

    end
end

%Defining the value for road length and merge length
roadLength = 300;
mergeLength = 30;
intersectionDistance = 100;
%% speed limits and acc/dec limits
vOut =[];
vOut(1:totalVehicles) = 15;
vMerge = [];
vMerge(1:totalVehicles) = 15;
u_min = -1;
u_max = 1;
v_max = 25;
v_min = 0;
%initialized T, R, D
T =[];% Schedule vector
R =[];% Release time
D =[];% Deadline
% parameters for safety constraints
gamma = 5;
phi = 0.2;
%% Safety at boundary: Refer to the paper for this condition 
safe = 0.5*u_min*timeHeadway^2+vMerge(1)*(timeHeadway-phi)-gamma;
fprintf(' Gamma with the defined time-headway %4.2f is %4.1f\n',timeHeadway,safe)
if safe<=0
    fprintf("change value of headway")
end
%%
conflict = zeros(totalVehicles,totalVehicles,totalZones);
pathInfo = zeros(totalVehicles,totalZones);
T = 0;
%initialize vehicle struct
for i = 1:totalVehicles
    x(i).Position=[0];
    x(i).Velocity=[vOut(i)];
    x(i).Control=[];
    x(i).Zone=[];
    x(i).Timestamp = TZeros(i);
end

%initialize intersection's geometry 
for i =1:totalZones
    zoneInfo(i).length = roadLength;
end

for i = 1:8
    zoneInfo(i).length = mergeLength;
end
zoneInfo(13).length = intersectionDistance;
zoneInfo(14).length = intersectionDistance;


%% Defining Path

%Origin = NB2
pathInfo(1,1:4) = [22,5,7,17];  %NB2 to SB2
pathInfo(16,1:6)=[22,5,14,2,1,9]; %NB2 to WB
pathInfo(17,1:5)=[22,5,14,2,15]; %NB2 to NB1
pathInfo(18,1:7)=[22,5,14,2,1,3,11]; %NB2 to SB1
pathInfo(19,1:5)=[22,5,7,8,19]; %NB2 to EB


%Origin = EB
pathInfo(5,1:7) = [20,6,5,14,2,1,9]; %EB to WB
pathInfo(20,1:8) = [20,6,5,14,2,1,3,11]; %EB to SB1
pathInfo(21,1:6) = [20,6,5,14,2,15]; %EB to NB1
pathInfo(22,1:5) = [20,6,5,7,17]; %EB to SB2
pathInfo(23,1:3) = [20,6,21]; %EB to NB2

%Origin = WB
pathInfo(3,1:7) = [10,3,4,13,7,8,19]; %WB to EB
pathInfo(9,1:5) = [10,3,4,2,15]; % WB to NB1
pathInfo(10,1:3) = [10,3,11]; % WB to SB1
pathInfo(11,1:8) = [10,3,4,13,7,8,6,21]; % WB to NB2
pathInfo(12,1:6) = [10,3,4,13,7,17]; % WB to SB2


%Origin = SB2
pathInfo(4,1:9) = [18,8,6,5,14,2,1,3,11];  %SB2 to SB1
pathInfo(8,1:4) = [18,8,6,21]; %SB2 to NB2
pathInfo(24,1:7) = [18,8,6,5,14,2,15];  %SB2 to NB1
pathInfo(25,1:8) = [18,8,6,5,14,2,1,9];  %SB2 to WB
pathInfo(26,1:3) = [18,8,19];  %SB2 to EB

%Origin = NB1
pathInfo(6,1:4) = [16,1,3,11]; %NB1 to SB1
pathInfo(27,1:9) = [16,1,3,4,13,7,8,6,21]; %NB1 to NB2
pathInfo(28,1:7) = [16,1,3,4,13,7,17]; %NB1 to NB2
pathInfo(29,1:8) = [16,1,3,4,13,7,8,19]; %NB1 to EB
pathInfo(30,1:3) = [16,1,9]; %NB1 to WB

%Origin = SB1
pathInfo(2,1:6) = [12,4,13,7,8,19]; %SB1 to EB
pathInfo(7,1:4) = [12,4,2,15]; %SB1 to NB1
pathInfo(13,1:7)=[12,4,13,7,8,6,21]; %SB1 to NB2
pathInfo(14,1:5)=[12,4,13,7,17]; %SB1 to SB2
pathInfo(15,1:5)=[12,4,2,1,9]; %SB1 to WB

%Creation  of conflict Set
conflictFinder(path,pathInfo,totalVehicles);

%Finding Earliest travel time for each segment for each CAV
ReleaseDeadlineFinder(path,pathInfo,totalVehicles,zoneInfo);
R(:,:)=round(R(:,:),2);
D(:,:)=round(D(:,:),2);


%Finding schedules
%%
if UPPERLEVELTEST 
    %Decentralized
    FIFO = false;
    [durationDEC,output] = scheduleFinderDec(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo);
    tfAvgDEC = 0;
    for ii=1:totalVehicles
        tfAvgDEC = max(T(ii,:)) - (T(ii,1)) + tfAvgDEC;
    end
    tfAvgDEC = tfAvgDEC/totalVehicles;
    
    
    FIFO = true;
    [durationFIFO,output] = scheduleFinderDec(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo);
    tfAvgFIFO = 0;
    for ii=1:totalVehicles
        tfAvgFIFO = max(T(ii,:)) - (T(ii,1)) + tfAvgFIFO;
    end
    tfAvgFIFO = tfAvgFIFO/totalVehicles;
    
    
    %%Centralized
    FIFO = false;
    [durationCEN, output] = scheduleFinderCentralized(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo);
    tfAvgCEN = 0;
    for ii=1:totalVehicles
        tfAvgCEN = max(T(ii,:)) - (T(ii,1)) + tfAvgCEN;
    end    
    tfAvgCEN = tfAvgCEN/totalVehicles;
    
    
    fprintf("%2d, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f, %2.4f \n ",totalVehicles, tfAvgDEC,tfAvgCEN,tfAvgFIFO, mean(durationDEC), durationCEN, mean(durationFIFO));
    return
end
%%
if CENTRALIZED
    fprintf("CENTRALIZED SCHEDULING\n")
    FIFO = false;
    [duration2, output] = scheduleFinderCentralized(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo);
    fprintf("Computation time: %2.5f \n",duration2);
    
else
    fprintf("DECENTRALIZED SCHEDULING\n")
    [duration,output] = scheduleFinderDec(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo);
    durAvg = mean(duration);
    fprintf("Computation time: %2.5f \n",durAvg);
end

tfAvg = 0;
for ii=1:totalVehicles
    tfAvg = max(T(ii,:)) - (T(ii,1)) + tfAvg;
end
tfAvg = tfAvg/totalVehicles;
fprintf("The average travel time of all CAVs, %2.2f\n", tfAvg)
T(:,:)=round(T(:,:),2);


%Controller
%Check for the type of zone
for i = 1:totalVehicles
    PathNumber = path(i);
    for j = 1:nnz(pathInfo(PathNumber,:))
        m = pathInfo(PathNumber,j);
        if abs(T(i,j+1) - T(i,j)-R(i,j))<0.01
            type(i,j) = "Time-optimal";
        elseif abs(T(i,j+1) - T(i,j)-D(i,j))<0.01
            type(i,j) = "Latest-Time";
        else
            type(i,j) = "Energy-optimal";
            disp(['Zone ' num2str(m) ' for the vehicle ' num2str(i) ' is not time-optimal'])
        end
    end
end
%TODO: Starting point for the time loop
%%
finish = 0;
tx = [0];

count = zeros(1,totalVehicles);
dt = 0;
RESTART = false;
for i = 1:totalVehicles
    solved(i).constants = nan;
    solved(i).done = false;
end
while dt < 100000 && tx(end) < max(T(:))
    
    dt = dt+1;
    if RESTART == true
        dt = 1;
        tx = [0];
        RESTART = false;
    end
    

    time = round(max((dt)*(0.01)),3);

    tx(end+1)=time;
    for i = 1:totalVehicles
        
        %Check the zone
        if tx(end)<= T(i,1)
            %CAV i has not entered yet
            continue
        end
        [m,j,finish] = zoneCheck(i,tx(end),pathInfo,path);
        if finish ==1
            continue
        end
   
        
        %time is the current time
        %if constraint is activated then
        x(i).Zone(end+1) = m;
        [x(i).Position(end+1),x(i).Velocity(end+1),x(i).Control(end+1),solved(i)] = controller(i,j,type,pathInfo,time,path,solved(i),zoneInfo);
        x(i).Timestamp(end+1) = time;
        %Check if control constraints becomes violated
        if ~isempty(x(i).Control) && count(i)<1 && CONSTRAINT
            if x(i).Control(end)< u_min -0.01
                fprintf('CAV %d violated Umin with %4.3f at time %4.2f \n',i,x(i).Control(end),time);
                count(i) = count(i) +1;
                ActivZone = x(i).Zone(end);
                PN = path(i);
                zoneNumber = find(pathInfo(PN,:) == ActivZone);
                type(i,zoneNumber) = "Umin";
                x = RESET(x,totalVehicles);
                RESTART = true;
                solved(i).done = false;
                solved(i).constants(end) = time;
                break
            end
            if x(i).Control(end)> u_max + 0.01
                fprintf('CAV %d violated umax with %4.3f at time %4.2f \n',i,x(i).Control(end),time);
                count(i) = count(i) +1;
                ActivZone = x(i).Zone(end);
                PN = path(i);
                zoneNumber = find(pathInfo(PN,:) == ActivZone);
                type(i,zoneNumber) = "Umax";
                x = RESET(x,totalVehicles);
                RESTART = true;
                solved(i).done = false;
                solved(i).constants(end) = time;
                break
            end
        end
        if ~isempty(x(i).Control) && count(i)<2 && CONSTRAINT
            if x(i).Control(end)> u_max + 0.01
                fprintf('CAV %d violated Umax again with %4.3f at time %4.2f \n',i,x(i).Control(end),time);
                count(i) = count(i) +1;
                ActivZone = x(i).Zone(end);
                PN = path(i);
                zoneNumber = find(pathInfo(PN,:) == ActivZone);
                
                if solved(i).constants(1)>0
                    type(i,zoneNumber) = "UminUnconsUmax";
                else
                    type(i,zoneNumber) = "UmaxUnconsUmin";
                end
                x = RESET(x,totalVehicles);
                solved(i).done = false;
                RESTART = true;
                break
            end
        end
        if ~isempty(x(i).Control) && count(i)<2 && CONSTRAINT
            if x(i).Control(end)< u_min - 0.01
                fprintf('CAV %d violated Umin again with %4.3f at time %4.2f \n',i,x(i).Control(end),time);
                count(i) = count(i) +1;
                ActivZone = x(i).Zone(end);
                PN = path(i);
                zoneNumber = find(pathInfo(PN,:) == ActivZone);
                
                if solved(i).constants(1)>0
                    type(i,zoneNumber) = "UminUnconsUmax";
                else
                    type(i,zoneNumber) = "UmaxUnconsUmin";
                end
                x = RESET(x,totalVehicles);
                solved(i).done = false;
                RESTART = true;
                break
            end
        end
        
        
        
        
        %Check if state constraint becomes violated
    end
    
end


%% Post- Processing
%Plot
if PLOT
    figure(1)
    for i=1:totalVehicles
        figure(1)
        subplot(4,1,path(i))
        plot(tx(find(tx==TZeros(i)):(length(x(i).Position)+find(tx==TZeros(i))-1)),x(i).Position(:));
        title(['Path',num2str(path(i))]);
        hold on
    end
    
    figure(2)
    for i=1:totalVehicles
        subplot(4,1,path(i))
        plot(tx(find(tx==TZeros(i)):(length(x(i).Velocity)+find(tx==TZeros(i))-1)),x(i).Velocity(:));
        title(['Path',num2str(path(i))]);
        hold on
    end
    
    figure(3)
    for i=1:totalVehicles
        figure
        plot(tx(find(tx==TZeros(i)):(length(x(i).Velocity)+find(tx==TZeros(i))-1)),x(i).Velocity(:));
        title(['CAV',num2str(i)]);
        hold on
    end

end
%%

%%PostProcessing
%%
if ANIMATIONPP
    M = PostProcessAnimation(x,T,path);
    if(SAVEVIDEO)
    date = datetime;
    date.Format = 'MMMM-d-yyyy-HH-mm-ss';
    name = string(date);
    newVid = VideoWriter(name, 'MPEG-4'); % New
    newVid.Quality = 100;
    open(newVid);
    writeVideo(newVid,M);
    pause(1);
    close(newVid)
    end
end


%Functions
%%

function PrintFig(title,file_label,AXIS,TICK)

axis(AXIS)
set(gca,...
    'Units','normalized',...
    'YTick',AXIS(3):TICK:AXIS(4),...
    'XTick',AXIS(1):10:AXIS(2),...
    'Position',[.15 .2 .75 .7],...
    'FontUnits','points',...
    'FontWeight','normal',...
    'FontSize',8,...
    'FontName','Times')
ylabel({title},...
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
box on
grid on
print(file_label,'-dpng')
end
function RearEndPosition(a,b,x,tx,TZeros,pathInfo,path)
v = sort([a,b]);
global conflict
a = conflict(v(2),v(1),:);
b = reshape(a,1,[]);
B = b(b~=0);
B = B(B~=4);%TODO: Fix it in future
disp(B);
if isempty(B)
    error('Path does not have conflict or it has lateral collision')
end
for i = v
    IndexInitR = find(x(i).Zone(:)== B(1),1);
    IndexEndR = find(x(i).Zone(:)== B(end),1,'last');
    if IndexInitR > IndexEndR
        error('This function only works if conflict zones of vehicle v1 and v2 are after each other');
    end
    IndexOrig = find(tx==TZeros(i),1);
    IndexInit = IndexOrig + IndexInitR - 1;
    IndexEnd = IndexOrig + IndexEndR - 1;
    [RelativePos,~,~,~] = mapGeometry(i,B(1),pathInfo,path,z);
    plot(tx(IndexInit:IndexEnd),x(i).Position(IndexInitR:IndexEndR)-RelativePos);
    hold on
    con = 0 ;
    g = [];
    for j = B
        con = con +1;
        InR = find(x(i).Zone(:)== j ,1,'last');
        PS = x(i).Position(InR)-RelativePos;
        g(end+1) = PS;
        g(end+1) = PS;
        disp([i,j,PS])
        line([0 50],[PS PS]);
        
        hold on
    end
    t = [0,50,50,0];
    g = [0,0,g];
    patch(t,g(1:4),'r','FaceAlpha',.2)
    patch(t,g(3:6),'g','FaceAlpha',.2)
    patch(t,g(5:8),'b','FaceAlpha',.2)
    
end

%%need to be completed
end
function RearEndPositionZone(zone,x,tx,TZeros,pathInfo,path)
%Checking the vehicle in the Zones
vehicle =[];


for i=1:length(x)
    PathNumber = path(i);
    if  any(pathInfo(PathNumber,:) == zone)
        vehicle(end+1) = i
    end
end
% check relative index for each vehicle when they enter the zone
for i = vehicle
    
    IndexInitR = find(x(i).Zone(:)== zone,1);
    IndexEndR = find(x(i).Zone(:)== zone,1,'last');
    IndexOrig = find(tx==TZeros(i),1);
    IndexInit = IndexOrig + IndexInitR - 1;
    IndexEnd = IndexOrig + IndexEndR - 1;
    [RelativePos,~,~,~] = mapGeometry(i,zone,pathInfo,path,z);
    plot(tx(IndexInit:IndexEnd),x(i).Position(IndexInitR:IndexEndR)-RelativePos);
    hold on
end
end
function x = RESET(x,totalVehicles)
global vOut
global T
for i = 1:totalVehicles
    x(i).Position=[0];
    x(i).Velocity=[vOut(i)];
    x(i).Control=[];
    x(i).Zone=[];
    x(i).Timestamp = T(i,1);
end
end



