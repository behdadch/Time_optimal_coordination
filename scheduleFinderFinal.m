function scheduleFinderFinal(path,pathInfo,totalVehicles,timeHeadway,TZeros)
global T
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
                disp(['$$$$$$$$$$$$$Go BACK ONE STEP $$$$$$$$$$$$$']);
                disp(['vehicleIndex and zones index are ',num2str(i),' and ',num2str(j)]);
                disp(['Release time is ',num2str(earliestEnter),' and Deadline is:  ',num2str(latestEnter)]);
                disp(['Previous Schedule ',num2str(T(i,j-1))]);
                disp(['$$$$$$$$$$$$$$$$$$$$$$$$$$']);
                
                mm = pathInfo(PathNumber,j-2);
                [pStart,pEnd,vStart,vEnd] = mapGeometry(i,mm,pathInfo,path);
                latestEnter2 = T(i,j-2) + deadline(vStart,vEnd,pStart,pEnd);
                if T(i,j-1) + timeHeadway <= latestEnter2
                T(i,j-1)= T(i,j-1) + timeHeadway;
                else
                    disp(['$$$$$$$$$$$$$Go BACK second STEP $$$$$$$$$$$$$']);
                    T(i,j-2)= T(i,j-2) + timeHeadway;
                    j=j-1;
                end
                
            end
        end
        
    end
end
end
