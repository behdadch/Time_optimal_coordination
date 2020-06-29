function M = PostProcessAnimation(vehiclesProfile,vehiclesSchedules, path,varargin)
%%it gets CAV's struct, schedule and path as input
%% global variables
global roadLength
global intersectionDistance
global mergeLength

%%
gamma = 5;
phi = 0.2;
%%

timeStep = 0.1;
flow_title = [];
horizon = max(vehiclesSchedules(:));
switch  nargin
    case 4
        flow_title = varargin{1};
    case 6
        horizon = varargin{1};
        timeStep = varargin{2}; 
end
%%
mapBuilder();
axis([0 2*roadLength + intersectionDistance+ 2*mergeLength  0 2*roadLength + mergeLength])
xlim = get(gca,'xlim')-170;
ylim = get(gca,'ylim')+15;
txt2 = 'Time:';
timetext=text(xlim(2),ylim(2),txt2);
timetext=text(xlim(2)-500,ylim(2),flow_title+" [veh/h]");


%%

time = 0:timeStep:horizon;
frame = 0; 
for t = time
    frame = frame +1;
    for i = 1:numel(vehiclesProfile)
        tpos = linspace( vehiclesSchedules(i,1), max(vehiclesSchedules(i,:)), numel(vehiclesProfile(i).Position));
        xpos = interp1(tpos, vehiclesProfile(i).Position, t);%%interpolate the position at simulation time = t
        vpos = interp1(tpos, vehiclesProfile(i).Velocity, t);
        
        if(xpos<0 || vpos<0)
            
            fprintf('Position or speed of CAV %d is negative \n',i);
        end 
        
        PathNumber = path(i);
        if PathNumber == 1 %&& finish == 0
            % path #1
            xx(i) = intersectionDistance+roadLength+5*mergeLength/4;
            yy(i) = 2*roadLength+mergeLength - xpos;
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),[0,0,1]);
            hold on;
        elseif PathNumber == 2
            % path #2
            if xpos<(roadLength+mergeLength/4)
                xx(i) = roadLength+3*mergeLength/4;
                yy(i) = xpos;
            else
                xx(i) = xpos+(mergeLength/2);
                yy(i) = roadLength+mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.k');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),[0,1,1]);
            hold on;
        elseif PathNumber == 3
            % path #3
            xx(i) = xpos;
            yy(i) = roadLength+mergeLength/4;
            hh(i) = plot(xx(i),yy(i),'.r');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i));
            hold on
        elseif PathNumber == 4
            if xpos< roadLength+3*mergeLength/4
                xx(i) = intersectionDistance+roadLength+7*mergeLength/4;
                yy(i) = xpos;
            elseif xpos< intersectionDistance+roadLength+9*mergeLength/4
                xx(i) = -xpos+(intersectionDistance+2*roadLength+5*mergeLength/2);
                yy(i) = roadLength+3*mergeLength/4;
            else
                xx(i) =  roadLength+mergeLength/4;
                yy(i) = -xpos+(intersectionDistance+2*roadLength+3*mergeLength);
            end
            hh(i) = plot(xx(i),yy(i),'.b');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),[1,1,1]);
            hold on
        end
    end
    
    
    
    %%clear the plot for the next time step
    txt = num2str(t);
    htext = text(xlim(2)+100,ylim(2),txt);
    M(frame) = getframe(gcf);
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
