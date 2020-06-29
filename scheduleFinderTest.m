function duration = scheduleFinderTest(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo)
global T
global R
global D
global vMerge 
global v_min
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
            [Sol,computationTime,exitflag] = MILPFinal(T(i,1),nnz(pathInfo(PathNumber,:)),R(i,:),D(i,:),order,schedule,timeHeadway,i);
            while  exitflag == -2 && vMerge(i)> v_min 
                fprintf('Decreasing vmerge speed from %4.2f to %4.2f for CAV: %d \n',vMerge(i),vMerge(i)-0.1,i);
                vMerge(i) = vMerge(i) - 0.1;
                ReleaseDeadlineFinder(path,pathInfo,totalVehicles,zoneInfo);
                R(:,:)=round(R(:,:),2);
                D(:,:)=round(D(:,:),2);
                [Sol,computationTime,exitflag] = MILPFinal(T(i,1),nnz(pathInfo(PathNumber,:)),R(i,:),D(i,:),order,schedule,timeHeadway,i);
            end 
            solution = Sol(1:nnz(pathInfo(PathNumber,:)))';
            T(i,2:nnz(pathInfo(PathNumber,:))+1)= solution;
            T(i,:);
            duration(i) = computationTime;
        end
        j = j+1;
        
    end
end
end
