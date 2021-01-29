function [duration,output] = scheduleFinderCentralized(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo)
global T
global R
global D
global vMerge
global v_min
global v_max
global v_out
for i=1:totalVehicles
    PathNumber = path(i);
    ConflictInfo(i).zoneInd = []; %%this vector grabs the zone index that conflict is happening
    ConflictInfo(i).schedule = []; %schedule of other CAV which CAV i has coupled safety constraint with
    ConflictInfo(i).rear = [];
    ConflictInfo(i).otherCAVID = [];
    T(i,1) = TZeros(i);
    for kk = 1 : i-1
        pathii = pathInfo(path(i),pathInfo(path(i),:)>0);
        pathKK = pathInfo(path(kk),pathInfo(path(kk),:)>0);
        [conflictZones,indicesA,indicesB] = intersect(pathii,pathKK,'stable');
        if (isempty(conflictZones)&& pathii(1)~=pathKK(1))
            %% there is no conflict with CAV kk
            continue;
        end
        indicesA = indicesA(indicesA>1)';
        indicesB = indicesB(indicesB>1)';
        otherCAVID = kk+zeros(1,nnz(indicesA));
        if (length(indicesA)>1)
            %There are two zones which are common between CAV i and kk
            %% now we need to check if theses two zones are after each other, if so --> rear-end will happen, so all these
            %zones should share the same binary variable
            if (indicesA(1) == indicesA(2)-1)
                %TODO: REAR-END
                rear = kk+zeros(1,nnz(indicesA));
                if(length(rear) ==length(pathii)-1)
                    rear= -rear; %% this means that CAV i and kk are in the same path and CAV i can not reach the end sooner than CAV kk --> therefore B_i = 0(refer to paper)
                end 
            else
                rear = zeros(1,nnz(indicesA));
            end
            
        elseif pathii(1)== pathKK(1) %they start on the same segment and then merge out
                    rear= -kk;
        else
            rear = 0;
        end
        
        ConflictInfo(i).zoneInd = [ConflictInfo(i).zoneInd, indicesA];
        ConflictInfo(i).schedule = [ConflictInfo(i).schedule, indicesB];
        ConflictInfo(i).otherCAVID = [ConflictInfo(i).otherCAVID, otherCAVID];
        ConflictInfo(i).rear = [ConflictInfo(i).rear,rear];
        
    end
end
%% Now we should pass this information to the solver

    [Sol,computationTime,exitflag,output] = MILPCentralized(T(:,1), path, pathInfo, R(:,:), D(:,:), ConflictInfo, timeHeadway, totalVehicles);
    fNew = 0;
    Sol;
    for ii = 1: totalVehicles
        fOld = fNew; % %total number
        conflictInfoZoneInd  = ConflictInfo(ii).zoneInd;
        PathNumber = path(ii);
        totalSegments = nnz(pathInfo(PathNumber,:));
        S = length(conflictInfoZoneInd);%create a binary variable for each safety constraint 
        fNew = totalSegments + S + fOld;
        solution = Sol(1+fOld:totalSegments+fOld);
        T(ii,2:totalSegments+1) = solution;
    end 
    %solution = Sol(1:nnz(pathInfo(PathNumber,:)))';
    %T(i,2:nnz(pathInfo(PathNumber,:))+1)= solution;
    duration = computationTime;
    


end
