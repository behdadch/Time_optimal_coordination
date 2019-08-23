function conflictFinder(pathInfo,totalVehicles)
%This function will find the conflict set for the vehicle i with respect to
%all the vehicles with smaller index and push it into the global variable
%"conflict" 
%%Author: Behdad Chalaki
%Advisor: Andreas Malikopoulos
%Phd Student at University of Delaware
%Information and decision Science Lab
%For more information, send an eamil to bchalaki@udel.edu
global conflict
for i=1:totalVehicles
    for j=1:totalVehicles 
        if ( j < i )
            x = nonzeros(intersect(pathInfo(i,:),pathInfo(j,:),'stable')).';
            size = length(x);
            conflict(i,j,1:size) = x;
        else
            continue  
        end   
    end
end
end 
