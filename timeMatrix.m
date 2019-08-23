function [b,c]= timeMatrix(pStart,pEnd,vStart,vEnd,tEnter,tExit,Acc)
%%Author: Behdad Chalaki
%Advisor: Andreas Malikopoulos
%Phd Student at University of Delaware
%Information and decision Science Lab
%For more information, send an eamil to bchalaki@udel.edu
global u_max
global u_min

if Acc == 1 % it is accelerating
    A =[tEnter 1;1 0];
    Y =[pStart-0.5*u_max*tEnter^2; vStart-u_max*tEnter];
else %it is decelerating
    A = [tExit 1;1 0];
    Y =[pEnd-0.5*u_min*(tExit)^2; vEnd-u_min*(tExit)];
   
end
X = A\Y;
b=X(1);
c=X(2);
end

