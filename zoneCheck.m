function [zone,index,finish]=zoneCheck(i,position,pathInfo)
finish = 0;
global roadLength
global mergeLength
L = roadLength;
S = mergeLength;
p=position;
if (rem(i,4) == 1)
    distance = [0,L,L+S,2*L+S];
elseif (rem(i,4) == 2)
    distance = [0,L,L+S/2,2*L+S/2,2*L+(3*S)/2,3*L+(3*S)/2];
elseif (rem(i,4) == 3)
    distance = [0,L,L+S,2*L+S,2*L+2*S,3*L+2*S];
elseif (rem(i,4) == 0)
    distance = [0,L,L+(3*S)/2,2*L+(3*S)/2,2*L+3*S,3*L+3*S];
end
if max(distance)< p
    zone = nan;
    index = nan; 
    finish = 1;
    return
end
index = find(distance == min(distance(distance>p)))-1;
  
try
zone = pathInfo(i,index);
catch
    disp(['i and index are ',num2str(i),' and ',num2str(index)]); 
end
end
