function M = PostProcessAnimation(vehiclesProfile,vehiclesSchedules, path,varargin)
%%it gets CAV's struct, schedule and path as input
%% global variables
global roadLength
global intersectionDistance
global mergeLength

%%
gamma = 4;
phi = 0.1;
%%

timeStep = 0.5; %% change the speed of animation
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
col = {[0, 0, 1],[1, 0, 0],[0, 0.75, 0.75],[0.75, 0, 0.75],[1,1,1],[0.8196,0.7098,0.0078]...
    ,[0/255 204/255 55/255],[1 1 1]	};

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
            c(i) = circle(xx(i),yy(i),HD(i),col{1});
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
            c(i) = circle(xx(i),yy(i),HD(i),col{6});
            hold on;
        elseif PathNumber == 3
            % path #3
            xx(i) = xpos;
            yy(i) = roadLength+mergeLength/4;
            hh(i) = plot(xx(i),yy(i),'.r');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{3});
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
            c(i) = circle(xx(i),yy(i),HD(i),col{4});
            hold on
        elseif PathNumber == 5
            xx(i) = 2*roadLength + intersectionDistance + 2*mergeLength - xpos;
            yy(i) = roadLength+3*mergeLength/4;
            hh(i) = plot(xx(i),yy(i),'.r');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{2});
            hold on
        elseif PathNumber == 6
            xx(i) = roadLength+mergeLength/4;
            yy(i) = 2*roadLength+mergeLength - xpos;
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{5});
            hold on;
        elseif PathNumber == 7 %&& finish == 0
            % path #1
            xx(i) = roadLength+3*mergeLength/4;
            yy(i) = xpos;
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{6});
            hold on;
        elseif PathNumber == 8 %&& finish == 0
            % path #1
            xx(i) = intersectionDistance+roadLength+7*mergeLength/4;
            yy(i) = xpos;
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{4});
            hold on;
        elseif PathNumber == 9
            if xpos< roadLength+3*mergeLength/4
                xx(i) = xpos;
                yy(i) = roadLength +mergeLength/4;
            else
                xx(i) = roadLength+3*mergeLength/4;
                yy(i) =  xpos - mergeLength/2;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{3});
            hold on;
            
        elseif PathNumber == 10
            if xpos< roadLength+mergeLength/4
                xx(i) = xpos;
                yy(i) = roadLength +mergeLength/4;
            else
                xx(i) = roadLength+mergeLength/4;
                yy(i) =  -(xpos - roadLength-mergeLength/4)+ roadLength+ mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{3});
            hold on;
        elseif PathNumber == 11
            if xpos< roadLength+intersectionDistance+7*mergeLength/4
                xx(i) = xpos;
                yy(i) = roadLength +mergeLength/4;
            else
                xx(i) = roadLength+intersectionDistance+7*mergeLength/4;
                yy(i) =  xpos - (roadLength+intersectionDistance+7*mergeLength/4)+ roadLength+ mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{3});
            hold on;
        elseif PathNumber == 12
            if xpos< roadLength+intersectionDistance+5*mergeLength/4
                xx(i) = xpos;
                yy(i) = roadLength +mergeLength/4;
            else
                xx(i) = roadLength+intersectionDistance+5*mergeLength/4;
                yy(i) =  -(xpos - (roadLength+intersectionDistance+5*mergeLength/4))+ roadLength+ mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{3});
            hold on;
        elseif PathNumber == 13
            if xpos < roadLength+mergeLength/4
                xx(i) = roadLength + 3*mergeLength/4;
                yy(i) = xpos;
            elseif xpos < roadLength+intersectionDistance+5*mergeLength/4
                xx(i) = (xpos-roadLength-mergeLength/4)+roadLength + 3*mergeLength/4;
                yy(i) =  roadLength + mergeLength/4;
            else
                xx(i) = roadLength+intersectionDistance+7*mergeLength/4;
                yy(i) =  (xpos-roadLength-intersectionDistance-5*mergeLength/4)+roadLength + mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{6});
            hold on;
        elseif PathNumber == 14
            if xpos < roadLength+mergeLength/4
                xx(i) = roadLength + 3*mergeLength/4;
                yy(i) = xpos;
            elseif xpos < roadLength+intersectionDistance+3*mergeLength/4
                xx(i) = xpos-(roadLength+mergeLength/4)+roadLength + 3*mergeLength/4;
                yy(i) =  roadLength + mergeLength/4;
            else
                xx(i) = roadLength+intersectionDistance+5*mergeLength/4;
                yy(i) =  -(xpos-(roadLength+intersectionDistance+3*mergeLength/4))+roadLength + mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{6});
            hold on;
        elseif PathNumber == 15
            if xpos < roadLength+3*mergeLength/4
                xx(i) = roadLength + 3*mergeLength/4;
                yy(i) = xpos;
            else
                xx(i) = -(xpos-(roadLength+3*mergeLength/4))+roadLength+3*mergeLength/4;
                yy(i) =  roadLength+3*mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{6});
            hold on;  
        elseif PathNumber == 16
            if xpos < roadLength+mergeLength/4
                xx(i) = roadLength + intersectionDistance+ 5*mergeLength/4;
                yy(i) = -xpos + 2*roadLength + mergeLength;
            else
                xx(i) = -(xpos-(roadLength+mergeLength/4))+roadLength + intersectionDistance+ 5*mergeLength/4;
                yy(i) =  roadLength+3*mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{1});
            hold on;
        elseif PathNumber == 17
            if xpos < roadLength+mergeLength/4
                xx(i) = roadLength + intersectionDistance+ 5*mergeLength/4;
                yy(i) = -xpos + 2*roadLength + mergeLength;
            elseif xpos < roadLength + intersectionDistance+ 3*mergeLength/4
                xx(i) = -(xpos-(roadLength+mergeLength/4))+roadLength + intersectionDistance+ 5*mergeLength/4;
                yy(i) =  roadLength+3*mergeLength/4;
            else
                xx(i) =  roadLength + 3*mergeLength/4;
                yy(i) =  (xpos-(roadLength + intersectionDistance+ 3*mergeLength/4))+roadLength + 3*mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{1});
            hold on;              
        elseif PathNumber == 18
            if xpos < roadLength+mergeLength/4
                xx(i) = roadLength + intersectionDistance+ 5*mergeLength/4;
                yy(i) = -xpos + 2*roadLength + mergeLength;
            elseif xpos < roadLength + intersectionDistance+ 5*mergeLength/4
                xx(i) = -(xpos-(roadLength+mergeLength/4))+roadLength + intersectionDistance+ 5*mergeLength/4;
                yy(i) =  roadLength+3*mergeLength/4;
            else
                xx(i) =  roadLength+mergeLength/4;
                yy(i) =  -(xpos-(roadLength + intersectionDistance+ 5*mergeLength/4))+roadLength+3*mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{1});
            hold on; 
        elseif PathNumber == 19
            if xpos < roadLength+3*mergeLength/4
                xx(i) = roadLength + intersectionDistance+ 5*mergeLength/4;
                yy(i) = -xpos + 2*roadLength + mergeLength;
            else
                xx(i) =(xpos-(roadLength+3*mergeLength/4))+roadLength+intersectionDistance+5*mergeLength/4;
                yy(i) =  roadLength+mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{1});
            hold on; 
        elseif PathNumber == 20
            if xpos < roadLength+intersectionDistance+7*mergeLength/4
                xx(i) = 2*roadLength + intersectionDistance + 2*mergeLength - xpos;
                yy(i) = roadLength+3*mergeLength/4;
            else
                xx(i) = roadLength+mergeLength/4;
                yy(i) =  -(xpos-(roadLength+intersectionDistance+7*mergeLength/4))+roadLength+3*mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{2});
            hold on;
        elseif PathNumber == 21
            if xpos < roadLength+intersectionDistance+5*mergeLength/4
                xx(i) = 2*roadLength + intersectionDistance + 2*mergeLength - xpos;
                yy(i) = roadLength+3*mergeLength/4;
            else
                xx(i) = roadLength+3*mergeLength/4;
                yy(i) =  (xpos-(roadLength+intersectionDistance+5*mergeLength/4))+roadLength+3*mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{2});
            hold on;
        elseif PathNumber == 22
            if xpos < roadLength+3*mergeLength/4
                xx(i) = 2*roadLength + intersectionDistance + 2*mergeLength - xpos;
                yy(i) = roadLength+3*mergeLength/4;
            else
                xx(i) = roadLength + intersectionDistance + 5*mergeLength/4;
                yy(i) =  -(xpos-(roadLength+3*mergeLength/4))+roadLength+3*mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{2});
            hold on;
        elseif PathNumber == 23
            if xpos < roadLength+mergeLength/4
                xx(i) = 2*roadLength + intersectionDistance + 2*mergeLength - xpos;
                yy(i) = roadLength+3*mergeLength/4;
            else
                xx(i) = roadLength + intersectionDistance + 7*mergeLength/4;
                yy(i) =  (xpos-(roadLength+mergeLength/4))+roadLength+3*mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{2});
            hold on;
        elseif PathNumber == 24
            if xpos< roadLength+3*mergeLength/4
                xx(i) = intersectionDistance+roadLength+7*mergeLength/4;
                yy(i) = xpos;
            elseif xpos< intersectionDistance+roadLength+7*mergeLength/4
                xx(i) = -(xpos-(roadLength+3*mergeLength/4))+intersectionDistance+roadLength+7*mergeLength/4;
                yy(i) = roadLength+3*mergeLength/4;
            else
                xx(i) =  roadLength+3*mergeLength/4;
                yy(i) = (xpos-(intersectionDistance+roadLength+7*mergeLength/4))+(roadLength+3*mergeLength/4);
            end
            hh(i) = plot(xx(i),yy(i),'.b');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{4});
            hold on            
        elseif PathNumber == 25
            if xpos< roadLength+3*mergeLength/4
                xx(i) = intersectionDistance+roadLength+7*mergeLength/4;
                yy(i) = xpos;
            else
                xx(i) = -(xpos-(roadLength+3*mergeLength/4))+intersectionDistance+roadLength+7*mergeLength/4;
                yy(i) = roadLength+3*mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.b');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{4});
            hold on
        elseif PathNumber == 26
            if xpos< roadLength+mergeLength/4
                xx(i) = intersectionDistance+roadLength+7*mergeLength/4;
                yy(i) = xpos;
            else
                xx(i) = (xpos-(roadLength+mergeLength/4))+intersectionDistance+roadLength+7*mergeLength/4;
                yy(i) = roadLength+mergeLength/4;
            end
            hh(i) = plot(xx(i),yy(i),'.b');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{4});
            hold on 
        elseif PathNumber == 27
            if xpos < roadLength+3* mergeLength/4
                xx(i) = roadLength+mergeLength/4;
                yy(i) = 2*roadLength+mergeLength - xpos;
            elseif xpos < roadLength+intersectionDistance+9* mergeLength/4
                xx(i) = xpos-(roadLength+3* mergeLength/4)+roadLength+mergeLength/4;
                yy(i) = roadLength+mergeLength/4;
            else 
                xx(i) = roadLength+intersectionDistance+7* mergeLength/4;
                yy(i) = xpos-(roadLength+intersectionDistance+9* mergeLength/4)+roadLength+mergeLength/4;
            end 
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{5});
            hold on;
        elseif PathNumber == 28
            if xpos < roadLength+3* mergeLength/4
                xx(i) = roadLength+mergeLength/4;
                yy(i) = 2*roadLength+mergeLength - xpos;
            elseif xpos < roadLength+intersectionDistance+7* mergeLength/4
                xx(i) = xpos-(roadLength+3* mergeLength/4)+roadLength+mergeLength/4;
                yy(i) = roadLength+mergeLength/4;
            else 
                xx(i) = roadLength+intersectionDistance+5* mergeLength/4;
                yy(i) = -(xpos-(roadLength+intersectionDistance+7* mergeLength/4))+roadLength+mergeLength/4;
            end 
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{5});
            hold on;
        elseif PathNumber == 29
            if xpos < roadLength+3* mergeLength/4
                xx(i) = roadLength+mergeLength/4;
                yy(i) = 2*roadLength+mergeLength - xpos;
            else
                xx(i) = xpos-(roadLength+3* mergeLength/4)+roadLength+mergeLength/4;
                yy(i) = roadLength+mergeLength/4;
            end 
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{5});
            hold on;
        elseif PathNumber == 30
            if xpos < roadLength+mergeLength/4
                xx(i) = roadLength+mergeLength/4;
                yy(i) = 2*roadLength+mergeLength - xpos;
            else
                xx(i) = -(xpos-(roadLength+mergeLength/4))+roadLength+mergeLength/4;
                yy(i) = roadLength+3*mergeLength/4;
            end 
            hh(i) = plot(xx(i),yy(i),'.m');
            HD(i) = gamma + vpos*phi;
            c(i) = circle(xx(i),yy(i),HD(i),col{5});
            hold on;             
            
            
        end
    end
    
    
    
    %%clear the plot for the next time step
    txt = num2str(t);
    htext = text(xlim(2)+100,ylim(2),txt);
    M(frame) = getframe(gcf);
    pause(0.1);
    delete(htext)
    for ii=1:length(hh)
        delete(hh(ii));
        if exist('c')
            delete(c(ii))
        end
    end
end
end
