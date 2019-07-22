function [zone,index,finish]=zoneCheck(vehicleIndex,position,pathInfo,path)
%This function will find zone that CAV is travelling through from the
%position of the CAV
finish = 0;
global roadLength
global mergeLength
L = roadLength;
S = mergeLength;
p=position;
x = vehicleIndex;
PathNumber = path(x);
if (PathNumber==1)
    distance = [0,L,L+0.5*S,L+S,2*L+S];
elseif (PathNumber==2)
    distance = [0,L,L+S/2,2*L+S/2,2*L+S,2*L+(3*S)/2,3*L+(3*S)/2];
elseif (PathNumber==3)
    distance = [0,L,L+S/2,L+S,2*L+S,2*L+(3*S)/2,2*L+2*S,3*L+2*S];
elseif (PathNumber==4)
    distance = [0,L,L+S/2,L+S,L+(3*S)/2,2*L+(3*S)/2,2*L+(4*S)/2,2*L+(5*S)/2,2*L+3*S,3*L+3*S];
end

if max(distance)< p
    zone = nan;
    index = nan; 
    finish = 1;
    return
end
index = find(distance == min(distance(distance>p)))-1;
  
try
zone = pathInfo(PathNumber,index);
catch
    disp(['vehicleIndex and zones index are ',num2str(vehicleIndex),' and ',num2str(index)]); 
end
end
