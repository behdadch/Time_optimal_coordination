%%Author: Behdad Chalaki
%Advisor: Andreas Malikopoulos
%Phd Student at University of Delaware
%Information and decision Science Lab
%For more information, send an eamil to bchalaki@udel.edu
%
close all
clear
clc
global u_min
global u_max
global vOut
global vMerge
global v_min
global conflict
global roadLength
global mergeLength
global T
global R
global D

%TODO: Defining the value for road length and merge length
roadLength = 400;
mergeLength = 30;
vOut = 25;
vMerge = 15;
u_min = -3;
u_max = 3;
v_min = 2;

totalZones = 16; % Number of zones
totalVehicles = 16; % Number of the vehicles
timeHeadway = 1.5;  % Safety Time Headway
conflict = zeros(totalVehicles,totalVehicles,totalZones);
pathInfo = zeros(totalVehicles,totalZones); %three vehicle
ANIMATION = true;
PLOT = true;
RANDOM = false;
%temp = zeros(totalVehicles,1);
T = 0;
for i = 1:totalVehicles
    x(i).Position =[0.00];
    x(i).Velocity =[vOut];
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
    TZeros = [0,1,1.5,1.7,1.65,2.5,3,3.2,3.15,4,4.5,4.7,4.65,5.5,6,6.2];
end

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
%Finding Earliest travel time for each segment for each CAV
ReleaseDeadlineFinder(pathInfo,totalVehicles);
R(:,:)=round(R(:,:),2);
D(:,:)=round(D(:,:),2);
%%
%Finding schedules
scheduleFinderTest(pathInfo,totalVehicles,timeHeadway,TZeros)
T(:,:)=round(T(:,:),2);

%%
%Check for the type of zone
for i = 1:totalVehicles
    for j = 1:nnz(pathInfo(i,:))
        m = pathInfo(i,j);
        if m == 1 || m == 2
            type(i,j) = "Merging-Zone";
        elseif abs(T(i,j+1) - T(i,j)-R(i,j))<0.01
            type(i,j) = "Time-optimal";
        elseif abs(T(i,j+1) - T(i,j)-D(i,j))<0.01
            type(i,j) = "Latest-Time";
        else
            type(i,j) = "Energy-optimal";
            disp(['Zone ' num2str(m) ' for the vehicle ' num2str(i) ' is not time-optimal'])
        end
    end
end
%%
finish = 0;
tx = [0];
if ANIMATION
    mapBuilder();
    txt2 = 'Time:';
    timetext=text(1000,720,txt2);
end


for dt=1:15000
    if ANIMATION
        time = max((dt)*(0.1));
    else
        time = round(max((dt)*(0.01)),2);
    end
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
        [m,j,finish] = zoneCheck(i,tx(end),pathInfo);
        if finish ==1
            continue
        end
        %time is the current time
        [x(i).Position(end+1),x(i).Velocity(end+1),x(i).Control(end+1),x(i).Zone(end+1)] = controller(i,j,type,pathInfo,x,time);
    end
    if ANIMATION
        txt = num2str(time);
        htext=text(1100,720,txt);
        M(dt) = getframe(gcf);
        pause(0.001);
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
    for i=1:4%totalVehicles
        plot(tx(find(tx==TZeros(i)):(length(x(i).Velocity)+find(tx==TZeros(i))-1)),x(i).Velocity(:));
        %title(['velocity',num2str(i)]);
        hold on
    end
    txt1 = 'Speed $(m/s)$';
    lbl1 = 'speed';
    ax1 = [0 60 0 45];
    PrintFig(txt1,lbl1,ax1,5);
    
    %      figure(2)
    %     for i=1:totalVehicles
    %         plot(tx(find(tx==TZeros(i)):(length(x(i).Control)+find(tx==TZeros(i))-1)),x(i).Control(:));
    %         %title(['velocity',num2str(i)]);
    %         hold on
    %     end
    %     txt2 = 'Control input $(m/s^2)$';
    %     lbl2 = 'control';
    %     ax2 = [0 60 -5 5];
    %     PrintFig(txt2,lbl2,ax2,1);
    %
    %figure(3)
    %RearEndPosition(6,3,x,tx,TZeros,pathInfo);
    %figure
    %RearEndPositionZone(11,x,tx,TZeros,pathInfo);
end
%%PostProcessing
%%Uncomment this for creating an animation 
%  newVid = VideoWriter('r=10Sim2', 'MPEG-4'); % New
%  newVid.Quality = 100;
%  open(newVid);
%  writeVideo(newVid,M);
%  close(newVid)


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








