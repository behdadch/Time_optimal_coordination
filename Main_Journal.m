close all
clear
clc
global u_min
global u_max
global vOut
global vMerge
global conflict
global roadLength
global mergeLength
global T

%TODO: Defining the value for road length and merge length
roadLength = 400;
mergeLength = 30;
vOut = 30;
vMerge = 15;
u_min = -1;
u_max = 1;
totalZones = 16; %Number of zones
totalVehicles = 16; %three vehicles
timeHeadway = 2.5;
conflict = zeros(totalVehicles,totalVehicles,totalZones);
pathInfo = zeros(totalVehicles,totalZones); %three vehicle
ANIMATION = false;
PLOT = false;
%temp = zeros(totalVehicles,1);
T = 0;
for i = 1:totalVehicles
    x(i).Position=[0];
    x(i).Velocity=[vOut];
    x(i).Control=[];
end
TZeros = [0,0,0,0,2,2,2,2,4,4,4,4,6,6,6,6];
%TODO: Order of the vehicle index should be added

%in pathInfo(i,j)-> i is vehicle index after order calculation and the j shows the
% zone that vehicle is in that.
pathInfo(1,1:3) = [5,2,15];  %Path of the vehicle 3
pathInfo(2,1:5) = [14,1,11,2,12]; %Path of the vehicle 2
pathInfo(3,1:5) = [10,1,11,2,12]; %Path of the vehicle 3
pathInfo(4,1:5) = [16,2,8,1,13];  %Path of the vehicle 4

pathInfo(5,1:3) = [5,2,15];
pathInfo(6,1:5) = [14,1,11,2,12];
pathInfo(7,1:5) = [10,1,11,2,12];
pathInfo(8,1:5) = [16,2,8,1,13];

pathInfo(9,1:3) = [5,2,15];
pathInfo(10,1:5) = [14,1,11,2,12];
pathInfo(11,1:5) = [10,1,11,2,12];
pathInfo(12,1:5) = [16,2,8,1,13];

pathInfo(13,1:3) = [5,2,15];
pathInfo(14,1:5) = [14,1,11,2,12];
pathInfo(15,1:5) = [10,1,11,2,12];
pathInfo(16,1:5) = [16,2,8,1,13];
%%
%Creation  of conflict Set
conflictFinder(pathInfo,totalVehicles);
%%
%Finding schedules
scheduleFinder3(pathInfo,totalVehicles,timeHeadway,TZeros)

%%
%Controller
%Check for the type of zone
for i = 1:totalVehicles
    for j = 1:nnz(pathInfo(i,:))
        m = pathInfo(i,j);
        [pStart,pEnd,vStart,vEnd] = mapGeometry(i,m,pathInfo);
        tCheck = T(i,j) + timeOptimal(vStart,vEnd,pStart,pEnd,m);
        if j ==nnz(pathInfo(i,:))
            %This is the last Segment
            type(i,j) = "Time-optimal";
            continue
        elseif m == 1 || m == 2
            type(i,j) = "Merging-Zone";
            continue
        end
        if T(i,j+1) > tCheck
            disp(['Zone ' num2str(m) ' for the vehicle ' num2str(i) ' is not time-optimal'])
            type(i,j) = "Energy-optimal";
        else
            type(i,j) = "Time-optimal";
        end
    end
end
%TODO: Starting point for the time loop
%%
T;
finish = 0;
tx=[0];
if ANIMATION
    mapBuilder();
    txt2 = 'Time:';
    timetext=text(1000,720,txt2);
end


for dt=1:3000
    time = (dt)*(0.1);
    tx(end+1)=time;
    for i = 1:totalVehicles
        if ANIMATION
            if rem(i,4) == 1 %&& finish == 0
                % path #1
                xx(i) = 2*roadLength+5*mergeLength/4;
                yy(i) = 2*roadLength+mergeLength- x(i).Position(end);
                hh(i) = plot(xx(i),yy(i),'.m');
                hold on;
            elseif rem(i,4) == 2
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
            elseif rem(i,4) == 3
                % path #3
                xx(i) = x(i).Position(end);
                yy(i) = roadLength+mergeLength/4;
                hh(i) = plot(xx(i),yy(i),'.r');
                hold on
            elseif rem(i,4) == 0
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
        [m,j,finish] = zoneCheck(i,x(i).Position(end),pathInfo);
        if finish ==1
            continue
        end
        %time is the current time
        [x(i).Position(end+1),x(i).Velocity(end+1),x(i).Control(end+1)] = controller(i,j,type,pathInfo,x,time);
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
    for i=1:totalVehicles
        plot(tx(find(tx==TZeros(i)):(length(x(i).Control)+find(tx==TZeros(i))-1)),x(i).Control(:));
        %title(['velocity',num2str(i)]);
        hold on
    end
end
%%




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
