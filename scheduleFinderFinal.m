function scheduleFinderFinal(path,pathInfo,totalVehicles,timeHeadway,TZeros)
global T
global timeHeadway
for i=1:totalVehicles
    %TODO:Also find the deadline for the vehicles to solve their scheduling
    %problem
    PathNumber = path(i);
    
    
    j=1;
    while j <= nnz(pathInfo(PathNumber,:))
        temp = [];
        k = 1;
        if j == 1
            T(i,j) = TZeros(i);
            j = j+1;
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
            latestEnter = T(i,j-1)+ deadline(vStart,vEnd,pStart,pEnd);
            earliestExit = earliestEnter + timeOptimal(vStart2,vEnd2,pStart2,pEnd2);
            
            try
                T(i,j)=MILP(temp,earliestEnter,latestEnter,timeHeadway);
                j = j+1;
            catch
                disp(['vehicleIndex and zones index are ',num2str(i),' and ',num2str(j)]);
                T(i,j-1)= T(i,j-1) + timeHeadway;
            end
        end
        
    end
end
end
