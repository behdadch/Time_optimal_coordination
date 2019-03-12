function scheduleFinder(pathInfo,totalVehicles,timeHeadway,TZeros)
global T
for i=1:totalVehicles
    for j=1:nnz(pathInfo(i,:))
        temp = 0;
        k = 1;
        if j == 1
            %TODO: Initial time for all the vehicle t zero
            T(i,j) = TZeros(i);
            %T(i,j) = 0;
        else
            m = pathInfo(i,j-1);
            n = pathInfo(i,j);
            while k <i
                if (any(pathInfo(k,:) == n))
                    x = find(pathInfo(k,:) == n);
                    temp(end+1) = T(k,x);
                end
                k = k+1;
            end
            [pStart,pEnd,vStart,vEnd] = mapGeometry(i,m,pathInfo);
            [pStart2,pEnd2,vStart2,vEnd2] = mapGeometry(i,n,pathInfo);
            earliestEnter = T(i,j-1) + timeOptimal(vStart,vEnd,pStart,pEnd,m);
            earliestExit = earliestEnter + timeOptimal(vStart2,vEnd2,pStart2,pEnd2,n);
            if isempty(min(temp(temp>0)))
                T(i,j) = earliestEnter;
            elseif (earliestExit <= min(temp(temp>0)))
                %vehicle i can enter earlier than vehicles with lower index
                T(i,j) = earliestEnter;
            else
                T(i,j)= max(max(temp(temp>0)) + timeHeadway , earliestEnter);
            end
        end
    end
end
end
