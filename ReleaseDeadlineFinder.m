function ReleaseDeadlineFinder(pathInfo,totalVehicles)
%%Author: Behdad Chalaki
%Advisor: Andreas Malikopoulos
%Phd Student at University of Delaware
%Information and decision Science Lab
%For more information, send an eamil to bchalaki@udel.edu
global R
global D
for i=1:totalVehicles
    j=1;
    while j <= nnz(pathInfo(i,:))
        m = pathInfo(i,j);
        [pStart,pEnd,vStart,vEnd] = mapGeometry(i,m,pathInfo);
        [R(i,j),~,~] = timeOptimal(vStart,vEnd,pStart,pEnd,pathInfo(i,j));
        [D(i,j),~,~,~,~] = deadline(vStart,vEnd,pStart,pEnd,pathInfo(i,j));
        j = j+1;
    end
end
end
