close all
clear
clc
global u_min
global u_max
global v_max
global vOut
global vMerge
global conflict
global roadLength
global mergeLength
global T
totalZones = 22; %Number of zones
totalVehicles = 4; %Number of vehicles
%TODO: Defining the value for road length and merge length
roadLength = 400;
mergeLength = 30;
vOut(1:totalVehicles) = 25;
vMerge = 15;
u_min = -3;
u_max = 3;
v_max = 35;

timeHeadway = 1.5;
conflict = zeros(totalVehicles,totalVehicles,totalZones);
pathInfo = zeros(totalVehicles,totalZones); 
ANIMATION = false;
PLOT = false;
RANDOM = false;
%temp = zeros(totalVehicles,1);
T = 0;
for i = 1:totalVehicles
    x(i).Position=[0];
    x(i).Velocity=[vOut(i)];
    x(i).Control=[];
    x(i).Zone=[];
end
if RANDOM
tmin = 0;
tmax = totalVehicles*timeHeadway*1.25;
n = totalVehicles;
TZeros = sort(tmin+rand(1,n)*(tmax-tmin));
TZeros = round(TZeros,2);
TZeros = TZeros - TZeros(1);
else
%TZeros = [0,1,1.5,1.7,1.65,2.5,3,3.2,3.15,4,4.5,4.7,4.65,5.5,6,6.2];
TZeros = [0,0.2,0.25,1.2];
end
%TODO: Order of the vehicle index should be added

%in pathInfo(i,j)-> i is vehicle index after order calculation and the j shows the
% zone that vehicle is in that.
path(1) = 1;
path(2) = 2;
path(3) = 3;
path(4) = 4;

%Defining Path
pathInfo(1,1:4) = [22,5,7,17];  %Path 1
pathInfo(2,1:6) = [12,4,13,7,8,19]; %Path 2
pathInfo(3,1:7) = [10,3,4,13,7,8,19]; %Path 3
pathInfo(4,1:9) = [18,8,6,5,14,2,1,3,11];  %Path 4


%%
%Creation  of conflict Set
conflictFinder(path,pathInfo,totalVehicles);
%%
%Finding schedules
scheduleFinder3(path,pathInfo,totalVehicles,timeHeadway,TZeros)
T(:,:)=round(T(:,:),2);
%%
%Controller
%Check for the type of zone
for i = 1:totalVehicles
    PathNumber = path(i);
    for j = 1:nnz(pathInfo(PathNumber,:))
        m = pathInfo(PathNumber,j);
        [pStart,pEnd,vStart,vEnd] = mapGeometry(i,m,pathInfo,path);
        tCheck = T(i,j) + timeOptimal(vStart,vEnd,pStart,pEnd);
        if j == nnz(pathInfo(PathNumber,:))
            %This is the last Segment
            type(i,j) = "Time-optimal";
            continue
        end
        if abs(T(i,j+1) - tCheck)>0.01
            disp(['Zone ' num2str(m) ' for the vehicle ' num2str(i) ' is not time-optimal','  T ',num2str(T(i,j+1)),'  R  ',num2str(tCheck)])
            type(i,j) = "Energy-optimal";
        else
            %disp(['Zone ' num2str(m) ' for the vehicle ' num2str(i) ' is time-optimal','  T ',num2str(T(i,j+1)),'  R  ',num2str(tCheck)])
            type(i,j) = "Time-optimal";
        end
    end
end
%TODO: Starting point for the time loop
%%
finish = 0;
tx=[0];
if ANIMATION
    mapBuilder();
    txt2 = 'Time:';
    timetext=text(1000,720,txt2);
end

count = 0;
dt =1;
RESTART = false;
while dt < 3000
    dt = dt+1;
    if RESTART==true
        dt=1; 
        RESTART = false;
    end 
        
    if ANIMATION
        time = max((dt)*(0.1));
    else
        time = round(max((dt)*(0.01)),2);
    end
    tx(end+1)=time;
    for i = 1:totalVehicles
        if ANIMATION
            PathNumber = path(i);
            if PathNumber == 1 %&& finish == 0
                % path #1
                xx(i) = 2*roadLength+5*mergeLength/4;
                yy(i) = 2*roadLength+mergeLength- x(i).Position(end);
                hh(i) = plot(xx(i),yy(i),'.m');
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
                hold on;
            elseif PathNumber == 3
                % path #3
                xx(i) = x(i).Position(end);
                yy(i) = roadLength+mergeLength/4;
                hh(i) = plot(xx(i),yy(i),'.r');
                hold on
            elseif PathNumber == 4
                if x(i).Position(end)< roadLength+3*mergeLength/4
                    xx(i) = 2*roadLength+7*mergeLength/4;
                    yy(i) = x(i).Position(end);
                elseif x(i).Position(end)< 2*roadLength+9*mergeLength/4
                    xx(i) = -x(i).Position(end)+(3*roadLength+5*mergeLength/2);
                    yy(i) = roadLength+3*mergeLength/4;
                else
                    xx(i) =  roadLength+mergeLength/4;
                    yy(i) = -x(i).Position(end)+(3*roadLength+3*mergeLength);
                end
                hh(i) = plot(xx(i),yy(i),'.b');
                hold on
            end
        end
        %Check the zone
        if tx(end)<= T(i,1)
            continue
        end
        %[m,j,finish] = zoneCheck(i,x(i).Position(end),pathInfo,path);
        [m,j,finish] = zoneCheck2(i,tx(end),pathInfo,path);
        if finish ==1
            continue
        end
        %time is the current time
        %if constraint is activated then 
        x(i).Zone(end+1) = m;

        [x(i).Position(end+1),x(i).Velocity(end+1),x(i).Control(end+1)] = controller(i,j,type,pathInfo,time,path);
        %Check if control constraints becomes violated 
        if length(x(i).Control)~= 0 && count<1
        if x(i).Control(end)< u_min 
            x(i).Control(end)
            count = count +1;
            ActivZone = x(i).Zone(end);
            type(i,ActivZone) = "Umin";
            x = RESET(x,totalVehicles);
            RESTART = true; 
            break
        end
        end 
        
        %Check if state constraint becomes violated
    end
    if ANIMATION
        txt = num2str(time);
        htext=text(1100,720,txt);
        M(dt) = getframe(gcf);
        pause(0.01);
        delete(htext)
        for i=1:totalVehicles
            delete(hh(i))
        end
        axis equal
        grid on
        box on
    end
    
end
%%
if PLOT
    width=4;%inch
    height=2;
    figure('Units','inches',...
        'Position',[0 0 width height],...
        'PaperPositionMode','auto');
    figure(1);
    for i=1:totalVehicles
        plot(tx(find(tx==TZeros(i)):(length(x(i).Velocity)+find(tx==TZeros(i))-1)),x(i).Velocity(:));
        %title(['velocity',num2str(i)]);
        hold on        
    end
    txt1 = 'Speed $(m/s)$';
    lbl1 = 'speed';
    ax1 = [0 60 0 45];
  %   PrintFig(txt1,lbl1,ax1,5);
     
      figure(2)
    for i=3%:totalVehicles
        plot(tx(find(tx==TZeros(i)):(length(x(i).Control)+find(tx==TZeros(i))-1)),x(i).Control(:));
        %title(['velocity',num2str(i)]);
        hold on       
    end
    txt2 = 'Control input $(m/s^2)$';
    lbl2 = 'control';
    ax2 = [0 60 -5 5];
%     PrintFig(txt2,lbl2,ax2,1);
%     
    %figure(3)
    %RearEndPosition(6,3,x,tx,TZeros,pathInfo);
    %figure
    %RearEndPositionZone(11,x,tx,TZeros,pathInfo);
end
%%PostProcessing

% newVid = VideoWriter('simulation2', 'MPEG-4'); % New
% newVid.Quality = 100;
% open(newVid);
% writeVideo(newVid,M);
%Functions


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
print(file_label,'-depsc2')
end
function RearEndPosition(a,b,x,tx,TZeros,pathInfo)
v = sort([a,b]);
global conflict
a = conflict(v(2),v(1),:);
b = reshape(a,1,[]);
B = b(b~=0);
B = B(B~=1);
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
[RelativePos,~,~,~] = mapGeometry(i,B(1),pathInfo);
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
function RearEndPositionZone(zone,x,tx,TZeros,pathInfo)
%Checking the vehicle in the Zones 
vehicle =[];
for i=1:length(pathInfo)
if  any(pathInfo(i,:) == zone)
    vehicle(end+1) = i;
end
end 
% check relative index for each vehicle when they enter the zone
for i = vehicle
IndexInitR = find(x(i).Zone(:)== zone,1);
IndexEndR = find(x(i).Zone(:)== zone,1,'last');
IndexOrig = find(tx==TZeros(i),1);
IndexInit = IndexOrig + IndexInitR - 1;
IndexEnd = IndexOrig + IndexEndR - 1;
[RelativePos,~,~,~] = mapGeometry(i,zone,pathInfo);
plot(tx(IndexInit:IndexEnd),x(i).Position(IndexInitR:IndexEndR)-RelativePos);
hold on
end
end
function x = RESET(x,totalVehicles)
global vOut
for i = 1:totalVehicles
    x(i).Position=[0];
    x(i).Velocity=[vOut(i)];
    x(i).Control=[];
    x(i).Zone=[];
end
end







%%
%TODO : fix control value at the switching point 



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
%maybe using structure would be a good idea
%%

% newVid = VideoWriter('simulation', 'MPEG-4'); % New
% newVid.Quality = 100;
% open(newVid);
% writeVideo(v,M);
%open(v);
%Functions
