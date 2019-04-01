function [zone,index,finish]=zoneCheck(vehicleIndex,position,pathInfo)
finish = 0;
global roadLength
global mergeLength
L = roadLength;
S = mergeLength;
p=position;
x = vehicleIndex;
if (rem(x,4)==1)
    distance = [0,L,L+0.5*S,L+S,2*L+S];
elseif (rem(x,4)==2)
    distance = [0,L,L+S/2,2*L+S/2,2*L+S,2*L+(3*S)/2,3*L+(3*S)/2];
elseif (rem(x,4)==3)
    distance = [0,L,L+S/2,L+S,2*L+S,2*L+(3*S)/2,2*L+2*S,3*L+2*S];
elseif (rem(x,4)==0)
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
zone = pathInfo(vehicleIndex,index);
catch
    disp(['vehicleIndex and zones index are ',num2str(vehicleIndex),' and ',num2str(index)]); 
end
end
