function [pStart,pEnd,vStart,vEnd] = mapGeometry(vehicleIndex,zoneNumber,pathInfo,path,z)
%This function wil return the entry and exit position of "zoneNumber" for the
%"vehicleIndex"
%zoneNumber should be the topological number of zone
%pathInfo should also be passed to this function
global roadLength
global mergeLength
global intersectionDistance
global vOut
global vMerge
L = roadLength;
D = intersectionDistance;
S = mergeLength;
PathNumber = path(vehicleIndex);
i = find(pathInfo(PathNumber,:) == zoneNumber);
x = vehicleIndex;



if (PathNumber==1)
    distance = [0,L,L+0.5*S,L+S,2*L+S];
elseif (PathNumber==2)
    distance = [0, L, L+S/2 ,L+S/2+D ,L+D+S ,L+D+(3*S)/2 ,2*L+D+(3*S)/2];
elseif (PathNumber==3)
    distance = [0,L, L+S/2 ,L+S, L+S+D, L+D+(3*S)/2, L+D+2*S, 2*L+D+2*S];
elseif (PathNumber==4)
    distance = [0, L, L+S/2, L+S, L+(3*S)/2, L+D+(3*S)/2, L+D+(4*S)/2, L+D+(5*S)/2, D+L+3*S, 2*L+D+3*S];
elseif (PathNumber==5)
    distance = [0,L, L+S/2 ,L+S, L+S+D, L+D+(3*S)/2, L+D+2*S, 2*L+D+2*S];
elseif (PathNumber==6)
    distance = [0,L,L+0.5*S,L+S,2*L+S];
elseif (PathNumber==7)
    distance = [0,L,L+0.5*S,L+S,2*L+S];
elseif (PathNumber==8)
    distance = [0,L,L+0.5*S,L+S,2*L+S];
    
    
    
end
%pathInfo(5,1:7) = [20,6,5,14,2,1,9]; %Path 5
%pathInfo(6,1:4) = [16,1,3,11]; %Path 6
%pathInfo(7,1:7) = [18,8,6,21]; %Path 7
%pathInfo(8,1:4) = [12,4,2,15]; %Path 8



pStart = distance(i);
pEnd = distance(i+1);

if (i==1)
    %it means that it is the first zone in the path
    vStart = vOut(vehicleIndex);
    vEnd = vMerge(vehicleIndex);
    %between two merging zones
elseif ( zoneNumber == 14 || zoneNumber == 13)
    vStart = vMerge(vehicleIndex);
    vEnd = vMerge(vehicleIndex);
elseif any([9,11,15,17,19,21]== zoneNumber)
    vStart = vMerge(vehicleIndex);
    vEnd = vOut(vehicleIndex);
else
    vStart = vMerge(vehicleIndex);
    vEnd = vMerge(vehicleIndex);
end

end