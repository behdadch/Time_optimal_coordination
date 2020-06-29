clear
close all
clc
%%
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
totalZones = 22; %Number of zones
totalPath =  4;
%%

ANIMATION = false;
PLOT = true;
RANDOM = false;
CONSTRAINT = true;
ANIMATIONPP = false;
INPUT = true;

%%
if INPUT
    importedData = importdata("C:\Users\Behdad\Dropbox\double intersection\Data\Seed1\Baseline-Input-400-15m-30s.csv");
    data = importedData.data;
    for i=1:length(data)
        index = find(data(:,2)==i);
        TZeros(i) = data(index,1);
        path(i) = data(index,3);
    end
    totalVehicles = length(data);
else
    
    if RANDOM
        tmin = 0;
        tmax = totalVehicles*timeHeadway*1.25;
        n = totalVehicles;
        TZeros = sort(tmin+rand(1,n)*(tmax-tmin));
        TZeros = round(TZeros,2);
        TZeros = TZeros - TZeros(1);
    else
        %%
        TZeros = [0,2.0,2.2,4.3,5.8,7.4,9.3,10.9,11.3,11.4,12.1,13.1,13.6,14.2,14.6,16.97,26,37,42,55]; %Random time generated for the Journal version
        %TZeros = [0,1,1.5,1.7,1.65,2.5,3,3.2,3.15,4,4.5,4.7,4.65,5.5,6,6.2];
        
    end
    %in pathInfo(i,j)-> i is vehicle index after order calculation and the j shows the
    % zone that vehicle is in that.
    totalVehicles = 20; %Number of vehicles
    
    for i = 1:totalVehicles
        path(i) = mod(i,totalPath);
        if path(i) == 0
            path(i) = totalPath;
        end
    end
end
%%


%Defining the value for road length and merge length
roadLength = 300;
mergeLength = 30;
intersectionDistance = 100;
vOut =[];
vOut(1:totalVehicles) = 15;
vMerge = [];
vMerge(1:totalVehicles) = 15;
u_min = -1;
u_max = 1;
v_max = 25;
v_min = 5;
timeHeadway = 1.5;
T =[];
R =[];
D =[];
gamma = 5;
phi = 0.2;
%% Safety at boundary
safe = 0.5*u_min*timeHeadway^2+vMerge(1)*(timeHeadway-phi)-gamma;
fprintf(' Gamma with the defined time-headway %4.2f is %4.1f\n',timeHeadway,safe)
if safe<=0
    fprintf("change value of headway")
end
%%
conflict = zeros(totalVehicles,totalVehicles,totalZones);
pathInfo = zeros(totalVehicles,totalZones);
T = 0;
for i = 1:totalVehicles
    x(i).Position=[0];
    x(i).Velocity=[vOut(i)];
    x(i).Control=[];
    x(i).Zone=[];
    x(i).Timestamp = TZeros(i);
end

for i =1:totalZones
    zoneInfo(i).length = roadLength;
end

for i = 1:8
    zoneInfo(i).length = mergeLength;
end
zoneInfo(13).length = intersectionDistance;
zoneInfo(14).length = intersectionDistance;

%%

%%

%Defining Path
pathInfo(1,1:4) = [22,5,7,17];  %Path 1
pathInfo(2,1:6) = [12,4,13,7,8,19]; %Path 2
pathInfo(3,1:7) = [10,3,4,13,7,8,19]; %Path 3
pathInfo(4,1:9) = [18,8,6,5,14,2,1,3,11];  %Path 4

% pathInfo(5,1:7) = [20,6,5,14,2,1,9]; %Path 5
% pathInfo(6,1:4) = [16,1,3,11]; %Path 6




%Creation  of conflict Set
conflictFinder(path,pathInfo,totalVehicles);
%%
%Finding Earliest travel time for each segment for each CAV
ReleaseDeadlineFinder(path,pathInfo,totalVehicles,zoneInfo);
R(:,:)=round(R(:,:),2);
D(:,:)=round(D(:,:),2);

%%
%Finding schedules
duration = scheduleFinderTest(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo);
max(duration);
mean(duration);
T(:,:)=round(T(:,:),2);

%%
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

if ANIMATION
    mapBuilder();
    axis([0 2*roadLength + intersectionDistance+ 2*mergeLength  0 2*roadLength + mergeLength])
    xlim = get(gca,'xlim')-170;
    ylim = get(gca,'ylim')+15;
    txt2 = 'Time:';
    timetext=text(xlim(2),ylim(2),txt2);
end
%%
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
    
    if ANIMATION
        time = max((dt)*(0.1));
    else
        time = round(max((dt)*(0.01)),3);
    end
    tx(end+1)=time;
    for i = 1:totalVehicles     
        
        %Check the zone
        if tx(end)<= T(i,1)
            %CAV i has not entered yet
            continue
        end
        %[m,j,finish] = zoneCheck(i,x(i).Position(end),pathInfo,path);
        [m,j,finish] = zoneCheck2(i,tx(end),pathInfo,path);
        if finish ==1
            continue
        end
        
        if ANIMATION
            PathNumber = path(i);
            if PathNumber == 1 %&& finish == 0
                % path #1
                xx(i) = intersectionDistance+roadLength+5*mergeLength/4;
                yy(i) = 2*roadLength+mergeLength- x(i).Position(end);
                hh(i) = plot(xx(i),yy(i),'.m');
                HD(i) = gamma + x(i).Velocity(end)*phi;
                c(i) = circle(xx(i),yy(i),HD(i),[0,0,1]);
                hold on;
            elseif PathNumber == 2
                % path #2
                if x(i).Position(end)<(roadLength+mergeLength/4)
                    xx(i) = roadLength+3*mergeLength/4;
                    yy(i) = x(i).Position(end);
                else
                    xx(i) = x(i).Position(end)+(mergeLength/2);
                    yy(i) = roadLength+mergeLength/4;
                end
                hh(i) = plot(xx(i),yy(i),'.k');
                HD(i) = gamma + x(i).Velocity(end)*phi;
                c(i) = circle(xx(i),yy(i),HD(i),[0,1,1]);
                hold on;
            elseif PathNumber == 3
                % path #3
                xx(i) = x(i).Position(end);
                yy(i) = roadLength+mergeLength/4;
                hh(i) = plot(xx(i),yy(i),'.r');
                HD(i) = gamma + x(i).Velocity(end)*phi;
                c(i) = circle(xx(i),yy(i),HD(i));
                hold on
            elseif PathNumber == 4
                if x(i).Position(end)< roadLength+3*mergeLength/4
                    xx(i) = intersectionDistance+roadLength+7*mergeLength/4;
                    yy(i) = x(i).Position(end);
                elseif x(i).Position(end)< intersectionDistance+roadLength+9*mergeLength/4
                    xx(i) = -x(i).Position(end)+(intersectionDistance+2*roadLength+5*mergeLength/2);
                    yy(i) = roadLength+3*mergeLength/4;
                else
                    xx(i) =  roadLength+mergeLength/4;
                    yy(i) = -x(i).Position(end)+(intersectionDistance+2*roadLength+3*mergeLength);
                end
                hh(i) = plot(xx(i),yy(i),'.b');
                HD(i) = gamma + x(i).Velocity(end)*phi;
                c(i) = circle(xx(i),yy(i),HD(i),[1,1,1]);
                hold on
            end
        end
        
        %time is the current time
        %if constraint is activated then
        x(i).Zone(end+1) = m;
        [x(i).Position(end+1),x(i).Velocity(end+1),x(i).Control(end+1),solved(i)] = controller(i,j,type,pathInfo,time,path,solved(i),zoneInfo);
        x(i).Timestamp(end+1) = time;
        %Check if control constraints becomes violated
        if length(x(i).Control)~= 0 && count(i)<1 && CONSTRAINT
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
        if length(x(i).Control)~= 0 && count(i)<2 && CONSTRAINT
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
        if length(x(i).Control)~= 0 && count(i)<2 && CONSTRAINT
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
    if ANIMATION
        txt = num2str(time);
        htext = text(xlim(2)+100,ylim(2),txt);
        M(dt) = getframe(gcf);
        pause(0.01);
        delete(htext)
        for ii=1:length(hh)
            delete(hh(ii));
            if exist('c')
                delete(c(ii))
            end
        end
    end
    
end


%% Post- Processing
%Plot
if PLOT
    %     width=4;%inch
    %     height=2;
    %     figure('Units','inches',...
    %         'Position',[0 0 width height],...
    %         'PaperPositionMode','auto');
    %%%%%%%%%%%%%%
    % figure(1)
%     figure
%     i =3;
%     plot(tx(find(tx==TZeros(i)):(length(x(i).Control)+find(tx==TZeros(i))-1)),x(i).Control(:),'k','LineWidth',1.2)
%     hold on
%     ylim = get(gca,'ylim');
%     tPl = T(i,:);
%     tPl = tPl(1:find(tPl == max(tPl)));
%     for i=1:length(tPl)
%         plot([tPl(i),tPl(i)],ylim,'--r')
%     end
%     xlabel('time')
%     ylabel('Control')
%     grid on
    %%%%%%%%%%%%%%%%%
    %     figure(1);
    %     for i=13:16%totalVehicles
    %         plot(tx(find(tx==TZeros(i)):(length(x(i).Velocity)+find(tx==TZeros(i))-1)),x(i).Velocity(:));
    %         %title(['velocity',num2str(i)]);
    %         hold on
    %     end
    %     txt1 = 'Speed $(m/s)$';
    %     lbl1 = 'speed';
    %     ax1 = [10 40 15 30];
    %     PrintFig(txt1,lbl1,ax1,5);
    %
    %     figure(2)
    %     for i=1:totalVehicles
    %         plot(tx(find(tx==TZeros(i)):(length(x(i).Control)+find(tx==TZeros(i))-1)),x(i).Control(:));
    %         %title(['Control input',num2str(i)]);
    %         hold on
    %     end
    %
    %         figure(3)
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
         %subplot(4,1,path(i))
        plot(tx(find(tx==TZeros(i)):(length(x(i).Velocity)+find(tx==TZeros(i))-1)),x(i).Velocity(:));
        %title(['Path',num2str(path(i))]);
        title(['CAV',num2str(i)]);
        hold on
    end
    %     txt2 = 'Control input $(m/s^2)$';
    %     lbl2 = 'ControlinputAllSched1';
    %     ax2 = [0 40 -2 2];
    %     PrintFig(txt2,lbl2,ax2,1);
    %
    %figure(3)
    %RearEndPosition(3,2,x,tx,TZeros,pathInfo,path);
    %figure
    %RearEndPositionZone(13,x,tx,TZeros,pathInfo,path);
end
%%

%%PostProcessing
%%
if ANIMATIONPP
    M = PostProcessAnimation(x,T,path,"1200");
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








%%%%%%%%%%%%%README%%%%%%%%%%%%
% axis([0 80 -3 3])
% set(gca,...
% 'Units','normalized',...
% 'YTick',-3:1:3,...
% 'XTick',0:10:80,...
% 'Position',[.15 .2 .75 .7],...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'FontSize',8,...
% 'FontName','Times')
% ylabel({'Control input $(m/s^2)$'},...
% 'FontUnits','points',...
% 'interpreter','latex',...
% 'FontSize',8,...
% 'FontName','Times')
% xlabel({'Time $(s)$'},...
% 'FontUnits','points',...
% 'FontWeight','normal',...
% 'FontSize',8,...
% 'FontName','Times')
%
% box on
% grid on
% print -depsc2 myplot.eps


%Time-Optimal Solution

%Energy-Optimal Solution
%%
%Readme
%min(A(A>0)) return minimum of nonzero array
%%

%  newVid = VideoWriter('r=10Sim2', 'MPEG-4'); % New
%  newVid.Quality = 100;
%  open(newVid);
%  writeVideo(newVid,M);
%  close(newVid)

%Functions
