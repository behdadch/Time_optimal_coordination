function [pStart,pEnd,vStart,vEnd] = mapGeometry(vehicleIndex,zoneNumber,pathInfo)
%This function wil return the entry and exit position of "zoneNumber" for the
%"vehicleIndex"
%zoneNumber should be the topological number of zone
%pathInfo should also be passed to this function
%%Author: Behdad Chalaki
%Advisor: Andreas Malikopoulos
%Phd Student at University of Delaware
%Information and decision Science Lab
%For more information, send an eamil to bchalaki@udel.edu
global roadLength
global mergeLength
global vOut 
global vMerge 
L = roadLength;
S = mergeLength;
i = find(pathInfo(vehicleIndex,:) == zoneNumber);
x = vehicleIndex;
if (rem(x,4)==1)
    distance = [0,L,L+S,2*L+S];
elseif (rem(x,4)==2)
    distance = [0,L,L+S/2,2*L+S/2,2*L+(3*S)/2,3*L+(3*S)/2];
elseif (rem(x,4)==3)
    distance = [0,L,L+S,2*L+S,2*L+2*S,3*L+2*S];
elseif (rem(x,4)==0)
    distance = [0,L,L+(3*S)/2,2*L+(3*S)/2,2*L+3*S,3*L+3*S];
end

pStart = distance(i);
pEnd = distance(i+1);

if (i==1)
    %it means that it is the first zone in the path
    vStart = vOut;
    vEnd = vMerge;
elseif ( zoneNumber == 8 || zoneNumber == 11)
    vStart = vMerge;
    vEnd = vMerge;
elseif any([4,6,7,12,13,15]== zoneNumber)
    vStart = vMerge;
    vEnd = vOut;
else
    vStart = vMerge;
    vEnd = vMerge;
end

end