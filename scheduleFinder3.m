function scheduleFinder3(path,pathInfo,totalVehicles,timeHeadway,TZeros)
global T
for i=1:totalVehicles
    %TODO:Also find the deadline for the vehicles to solve their scheduling
    %problem
    PathNumber = path(i);
    for j=1:nnz(pathInfo(PathNumber,:))
        temp = [];
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
                    temp(end+1) = T(k,x);
                end
                k = k+1;
            end
            [pStart,pEnd,vStart,vEnd] = mapGeometry(i,m,pathInfo,path);
            [pStart2,pEnd2,vStart2,vEnd2] = mapGeometry(i,n,pathInfo,path);
            earliestEnter = T(i,j-1) + timeOptimal(vStart,vEnd,pStart,pEnd);
            earliestExit = earliestEnter + timeOptimal(vStart2,vEnd2,pStart2,pEnd2);

            T(i,j)=MILP(temp,earliestEnter,timeHeadway);
        end
        
    end
end
end
