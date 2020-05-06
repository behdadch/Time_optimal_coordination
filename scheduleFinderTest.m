function scheduleFinderTest(path,pathInfo,totalVehicles,timeHeadway,TZeros)
global T
global R
global D
for i=1:totalVehicles
    PathNumber = path(i);
    j=1;
    ConflictInfo.order = [];
    ConflictInfo.schedule = [];
    while j <= nnz(pathInfo(PathNumber,:))
        k = 1;
        if j == 1
            T(i,j) = TZeros(i);
        else
            m = pathInfo(PathNumber,j-1);
            n = pathInfo(PathNumber,j);
            while k <i
                PN = path(k);
                if (any(pathInfo(PN,:) == n))
                    x = find(pathInfo(PN,:) == n);
                    ConflictInfo.schedule(end+1) = T(k,x);
                    ConflictInfo.order(end+1)= j;
                end
                k = k+1;
            end
        end
        if j == nnz(pathInfo(PathNumber,:))
            %MILP should be solved
            order = ConflictInfo.order;
            schedule = ConflictInfo.schedule;
            Sol = MILPFinal(T(i,1),nnz(pathInfo(PathNumber,:)),R(i,:),D(i,:),order,schedule,timeHeadway);
            T(i,2:nnz(pathInfo(PathNumber,:))+1)=Sol;
            T(i,:)
        end
        j = j+1;
        
    end
end
end
