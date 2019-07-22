function [zone,index,finish]=zoneCheck2(vehicleIndex,time,pathInfo,path)
%This function will find zone that CAV is travelling through from the
%schedule of the CAV
finish = 0;
global T

x = vehicleIndex;
PathNumber = path(x);
j = find( T(x,:) == max(T(x,:)) );
m = pathInfo(PathNumber,j);
[pStart,pEnd,vStart,vEnd] = mapGeometry(x,m,pathInfo,path);
[tf,~,~,~,~] = timeOptimal(vStart,vEnd,pStart,pEnd);
if max(T(x,:))+ tf < time %CAV exited the control zone
    zone = nan;
    index = nan; 
    finish = 1;
    return
elseif max(T(x,:))<= time %CAV is at the last zone
index = j;    
else
index = find(T(x,:)== min(T(x,T(x,:)>time)))-1;
end

try
zone = pathInfo(PathNumber,index);
catch
    disp(['vehicleIndex and zones index are ',num2str(vehicleIndex),' and ',num2str(index)]); 
end
end
