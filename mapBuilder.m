function mapBuilder()
%%Author: Behdad Chalaki
%Advisor: Andreas Malikopoulos
%Phd Student at University of Delaware
%Information and decision Science Lab
%For more information, send an eamil to bchalaki@udel.edu
global roadLength
global mergeLength

%Horizontal Lines 
t1=linspace(0,3*roadLength+2*mergeLength,1000);
y1(1:1000)=roadLength;
y2(1:1000)=roadLength+mergeLength;
ym(1:1000)=roadLength+mergeLength/2;

%Vertical Lines 
t2=linspace(0,2*roadLength+mergeLength,1000);
x1(1:1000)=roadLength;
x1m(1:1000)=roadLength+mergeLength/2;
x2(1:1000)=roadLength+mergeLength;
x3(1:1000)=2*roadLength+mergeLength;
x3m(1:1000)=2*roadLength+3*mergeLength/2;
x4(1:1000)=2*roadLength+2*mergeLength;

%Ploting lines
figure  
plot(t1,y1,'-k');
hold on 
plot(t1,y2,'-k');
hold on 
plot(t1,ym,'--r');
hold on 
plot (x1,t2,'-k');
hold on 
plot (x1m,t2,'--r');
hold on 
plot (x2,t2,'-k');
hold on 
plot (x3,t2,'-k');
hold on 
plot (x3m,t2,'--r');
hold on 
plot (x4,t2,'-k');
hold on 
axis equal
end
