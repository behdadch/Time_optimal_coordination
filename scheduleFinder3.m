function scheduleFinder3(pathInfo,totalVehicles,timeHeadway,TZeros)
global T
for i=1:totalVehicles
    %TODO:Also find the deadline for the vehicles to solve their scheduling
    %problem
    for j=1:nnz(pathInfo(i,:))
        temp = [];
        k = 1;
        if j == 1
            T(i,j) = TZeros(i);
        else
            m = pathInfo(i,j-1);
            n = pathInfo(i,j);
            o = pathInfo(i,j+1);
            [pStart,pEnd,vStart,vEnd]     =  mapGeometry(i,m,pathInfo);
            [pStart2,pEnd2,vStart2,vEnd2] =  mapGeometry(i,n,pathInfo);
            earliestEnter = T(i,j-1) + timeOptimal(vStart,vEnd,pStart,pEnd,m);
            processTime   = timeOptimal(vStart2,vEnd2,pStart2,pEnd2,n);
            
            if n==1 || n==2
                extra = true;
            end
            
            while k <i
                if (any(pathInfo(k,:) == n))
                    x = find(pathInfo(k,:) == n);
                    temp(end+1) = T(k,x);
                end
                if (any(pathInfo(k,:) == o)&& o~=0 && extra)
                    y = find(pathInfo(k,:) == o);
                    temp(end+1) = T(k,y)-processTime;
                end
                k = k+1;
            end

            if m==1 || m==2
                T(i,j)= T(i,j-1) + timeOptimal(vStart,vEnd,pStart,pEnd,m);
            else 
                T(i,j)=MILP(temp,earliestEnter,timeHeadway);
            end
        end
        
    end
end
end
