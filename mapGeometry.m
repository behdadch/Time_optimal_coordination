function [pStart,pEnd,vStart,vEnd] = mapGeometry(vehicleIndex,zoneNumber,pathInfo,path)
%This function wil return the entry and exit position of "zoneNumber" for the
%"vehicleIndex"
%zoneNumber should be the topological number of zone
%pathInfo should also be passed to this function
global roadLength
global mergeLength
global vOut 
global vMerge 
L = roadLength;
S = mergeLength;
PathNumber = path(vehicleIndex);
i = find(pathInfo(PathNumber,:) == zoneNumber);
x = vehicleIndex;
if (PathNumber==1)
    distance = [0,L,L+0.5*S,L+S,2*L+S];
elseif (PathNumber==2)
    distance = [0,L,L+S/2,2*L+S/2,2*L+S,2*L+(3*S)/2,3*L+(3*S)/2];
elseif (PathNumber==3)
    distance = [0,L,L+S/2,L+S,2*L+S,2*L+(3*S)/2,2*L+2*S,3*L+2*S];
elseif (PathNumber==4)
    distance = [0,L,L+S/2,L+S,L+(3*S)/2,2*L+(3*S)/2,2*L+(4*S)/2,2*L+(5*S)/2,2*L+3*S,3*L+3*S];
end
pStart = distance(i);
pEnd = distance(i+1);

if (i==1)
    %it means that it is the first zone in the path
    vStart = vOut(vehicleIndex);
    vEnd = vMerge;
    %between two merging zones
elseif ( zoneNumber == 14 || zoneNumber == 13)
    vStart = vMerge;
    vEnd = vMerge;
elseif any([9,11,15,17,19,21]== zoneNumber)
    vStart = vMerge;
    vEnd = vOut(vehicleIndex);
else
    vStart = vMerge;
    vEnd = vMerge;
end

end