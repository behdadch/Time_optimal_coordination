function [duration,output] = scheduleFinderDec(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo)
global T
global R
global D
global vMerge
global v_min
global v_max
global v_out
for i=1:totalVehicles
    PathNumber = path(i);
    ConflictInfo.zoneInd = []; %%this vector grabs the zone index that conflict is happening
    ConflictInfo.schedule = []; %schedule of other CAV which CAV i has coupled safety constraint with
    ConflictInfo.rear = [];
    T(i,1) = TZeros(i);
    for kk = 1 : i-1
        pathii = pathInfo(path(i),pathInfo(path(i),:)>0);
        pathKK = pathInfo(path(kk),pathInfo(path(kk),:)>0);
        [conflictZones,indicesA,indicesB] = intersect(pathii,pathKK,'stable');
        if (isempty(conflictZones) && pathii(1)~=pathKK(1))
            %% there is no conflict with CAV kk
            continue;
        end
        
        indicesA = indicesA(indicesA>1)';
        indicesB = indicesB(indicesB>1)';
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
            
        elseif pathii(1)==pathKK(1) %they start on the same segment and then merge out
            rear= -kk;
        else
            rear = 0;
        end
        
        ConflictInfo.zoneInd = [ConflictInfo.zoneInd, indicesA];
        ConflictInfo.schedule = [ConflictInfo.schedule, T(kk,indicesB)];
        ConflictInfo.rear = [ConflictInfo.rear,rear];
        
    end
    %MILP should be solved
    %     if i ==7
    %         ConflictInfo.rear
    %         ConflictInfo.zoneInd
    %     end
    %%
    zoneInd = ConflictInfo.zoneInd;
    schedule = ConflictInfo.schedule;
    rear = ConflictInfo.rear;
    [Sol,computationTime,exitflag,output] = MILPDec(T(i,1),nnz(pathInfo(PathNumber,:)),R(i,:),D(i,:),zoneInd,schedule,rear,timeHeadway,i);
    while  exitflag == -2 && vMerge(i)> v_min
        fprintf('Decreasing vmerge speed from %4.2f to %4.2f for CAV: %d \n',vMerge(i),vMerge(i)-0.1,i);
        vMerge(i) = vMerge(i) - 0.1;
        %T(i,1) = T(i,1)+0.5;
        %v_min = 0.1;
        %v_out = v_out + 0.1;
        ReleaseDeadlineFinder(path,pathInfo,totalVehicles,zoneInfo);
        R(:,:)=round(R(:,:),2);
        D(:,:)=round(D(:,:),2);
        [Sol,computationTime,exitflag] = MILPDec(T(i,1),nnz(pathInfo(PathNumber,:)),R(i,:),D(i,:),zoneInd,schedule,rear,timeHeadway,i);
    end
    %     if i==7
    %         Sol
    %     end
    solution = Sol(1:nnz(pathInfo(PathNumber,:)))';
    T(i,2:nnz(pathInfo(PathNumber,:))+1)= solution;
    duration(i) = computationTime;
    
    %%
    % % % %             T(i,1) = TZeros(i);
    % % % %             T(i,2:nnz(pathInfo(PathNumber,:))+1)=-i+zeros(1,nnz(pathInfo(PathNumber,:)));
    % % % %             duration(i) = -1;
    % % % %             i
    % % % %             ConflictInfo.order
end
end
