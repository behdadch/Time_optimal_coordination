function scheduleFinderTest(pathInfo,totalVehicles,timeHeadway,TZeros)
%%Author: Behdad Chalaki
%Advisor: Andreas Malikopoulos
%Phd Student at University of Delaware
%Information and decision Science Lab
%For more information, send an eamil to bchalaki@udel.edu
global T
global R
global D
for i=1:totalVehicles
    j=1;
    ConflictInfo.order = [];
    ConflictInfo.schedule = [];
    while j <= nnz(pathInfo(i,:))
        k = 1;
        if j == 1
            T(i,j) = TZeros(i);
        else
            m = pathInfo(i,j-1);
            n = pathInfo(i,j);
            while k <i
                if (any(pathInfo(k,:) == n))
                    x = find(pathInfo(k,:) == n);
                    ConflictInfo.schedule(end+1) = T(k,x);
                    ConflictInfo.order(end+1)= j;
                end
                k = k+1;
            end
        end
        if j == nnz(pathInfo(i,:))
            %MILP should be solved
            order = ConflictInfo.order;
            schedule = ConflictInfo.schedule;
            Sol = MILPFinal(T(i,1),nnz(pathInfo(i,:)),R(i,:),D(i,:),order,schedule,timeHeadway);
            T(i,2:nnz(pathInfo(i,:))+1)=Sol;
            T(i,:)
        end
        j = j+1;
        
    end
end
end
