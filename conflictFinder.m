function conflictFinder(path,pathInfo,totalVehicles)
%This function will find the conflict set for the vehicle i with respect to
%all the vehicles with smaller indices (FCFS) and push it into the global variable
%"conflict" 
global conflict
for i=1:totalVehicles
    for j=1:totalVehicles 
        if ( j < i )
            k = path(i);
            kk = path(j);
            x = nonzeros(intersect(pathInfo(k,:),pathInfo(kk,:),'stable')).';
            size = length(x);
            conflict(i,j,1:size) = x;
        else
            continue  
        end   
    end
end
end 
