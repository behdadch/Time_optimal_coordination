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
elseif (PathNumber==9)
    distance = [0,L,L+0.5*S,L+S,L+1.5*S,2*L+1.5*S];    
elseif (PathNumber==10)
    distance = [0,L,L+0.5*S,2*L+0.5*S];   
elseif (PathNumber==11)
    distance = [0, L, L+0.5*S, L+S, L+S+D, L+D+1.5*S, L+D+2*S, L+D+2.5*S, 2*L+D+2.5*S];      
elseif (PathNumber==12)
    distance = [0, L, L+0.5*S, L+S, L+S+D, L+D+1.5*S, 2*L+D+1.5*S];
elseif (PathNumber==13)
    distance = [0, L, L+0.5*S, L+0.5*S+D, L+D+S, L+D+1.5*S, L+D+2*S, 2*L+D+2*S];
elseif (PathNumber==14)
    distance = [0, L, L+0.5*S, L+0.5*S+D, L+D+S, 2*L+D+S];
elseif (PathNumber==15)
    distance = [0, L, L+0.5*S, L+S, L+1.5*S, 2*L+1.5*S];
elseif (PathNumber==16)
    distance = [0, L, L+0.5*S, L+D+0.5*S, L+D+S, L+D+1.5*S, 2*L+D+1.5*S];
elseif (PathNumber==17)
    distance = [0, L, L+0.5*S, L+0.5*S+D, L+D+S, 2*L+D+S];
elseif (PathNumber==18)
    distance = [0, L, L+0.5*S, L+0.5*S+D, L+D+S, L+D+1.5*S, L+D+2*S, 2*L+D+2*S];
elseif (PathNumber==19)
    distance = [0, L, L+0.5*S, L+S, L+1.5*S, 2*L+1.5*S];
elseif (PathNumber==20)
    distance = [0, L, L+0.5*S, L+S, L+S+D, L+D+1.5*S, L+D+2*S, L+D+2.5*S, 2*L+D+2.5*S];    
elseif (PathNumber==21)    
    distance = [0, L, L+0.5*S, L+S, L+S+D, L+D+1.5*S, 2*L+D+1.5*S];
elseif (PathNumber==22)    
    distance = [0, L, L+0.5*S, L+S, L+1.5*S, 2*L+1.5*S];
elseif (PathNumber==23)
    distance = [0,L,L+0.5*S,2*L+0.5*S];
elseif (PathNumber==24)
    distance = [0, L, L+0.5*S, L+S, L+1.5*S, L+1.5*S+D, L+2*S+D, 2*L+2*S+D];
elseif (PathNumber==25)
    distance = [0, L, L+0.5*S, L+S, L+1.5*S, L+1.5*S+D, L+2*S+D, L+2.5*S+D 2*L+2*S+D];
elseif (PathNumber==26)
    distance = [0,L,L+0.5*S,2*L+0.5*S];
elseif (PathNumber==27)
    distance = [0, L, L+S/2, L+S, L+(3*S)/2, L+D+(3*S)/2, L+D+(4*S)/2, L+D+(5*S)/2, D+L+3*S, 2*L+D+3*S];
elseif (PathNumber==28)
    distance = [0, L, L+S/2, L+S, L+(3*S)/2, L+D+(3*S)/2, L+D+(4*S)/2, 2*L+D+2*S];
elseif (PathNumber==29)
    distance = [0, L, L+S/2, L+S, L+(3*S)/2, L+D+(3*S)/2, L+D+(4*S)/2,  L+D+2.5*S, 2*L+D+2*S];
elseif (PathNumber==30)
    distance = [0,L,L+0.5*S,2*L+0.5*S];    
end


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