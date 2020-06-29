function mapBuilder()
global roadLength
global mergeLength
global intersectionDistance


%Horizontal Lines 
t1=linspace(0,2*roadLength+intersectionDistance+2*mergeLength,1000);
y1(1:1000)=roadLength;
y2(1:1000)=roadLength+mergeLength;
ym(1:1000)=roadLength+mergeLength/2;

%Vertical Lines 
t2=linspace(0,2*roadLength+mergeLength,1000);
x1(1:1000)=roadLength;
x1m(1:1000)=roadLength+mergeLength/2;
x2(1:1000)=roadLength+mergeLength;
x3(1:1000)=intersectionDistance + roadLength+mergeLength;
x3m(1:1000)=intersectionDistance + roadLength+3*mergeLength/2;
x4(1:1000)=roadLength+intersectionDistance+2*mergeLength;

%Ploting lines
figure  
plot(t1,y1,'-k','LineWidth',2);
hold on 
plot(t1,y2,'-k','LineWidth',2);
hold on 
plot(t1,ym,'--y','LineWidth',2);
hold on 
plot (x1,t2,'-k','LineWidth',2);
hold on 
plot (x1m,t2,'--y','LineWidth',2);
hold on 
plot (x2,t2,'-k','LineWidth',2);
hold on 
plot (x3,t2,'-k','LineWidth',2);
hold on 
plot (x3m,t2,'--y','LineWidth',2);
hold on 
plot (x4,t2,'-k','LineWidth',2);
hold on 
axis equal

green = [0,1,0];

patch([0 0 x1(end) x1(end)],[0, y1(end) y1(end), 0],green)
patch([x2(end) x2(end) x3(end) x3(end)],[0, y1(end) y1(end), 0],green)
patch([x4(end) x4(end) x4(end)+roadLength x4(end)+roadLength],[0, y1(end) y1(end), 0],green)

p1 =roadLength+mergeLength;
patch([0 0 x1(end) x1(end)],[p1, y1(end)+p1 y1(end)+p1, p1],green)
patch([x2(end) x2(end) x3(end) x3(end)],[p1, y1(end)+p1 y1(end)+p1, p1],green)
patch([x4(end) x4(end) x4(end)+roadLength x4(end)+roadLength],[p1, y1(end)+p1 y1(end)+p1, p1],green)
% 
gray = [17 17 17]/255;
set(gca,'Color',gray)
end
