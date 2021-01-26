function [Schedule, computationTime, exitflag] = MILPCentralized(Tinitial, path, pathInfo, Release,Deadline, ConflictInfo, timeHeadway, totalVehicles)

%% Initialization
solver = "IBMCPLEX";
M = 100;% this is a big number for the big M method to handle disjunctive (OR) constraints
f = [];
constrNum =0;
intcon = [];
for ii = 1:totalVehicles
    conflictInfoZoneInd  = ConflictInfo(ii).zoneInd;
    PathNumber = path(ii);
    totalSegments = nnz(pathInfo(PathNumber,:));
    
    S = length(conflictInfoZoneInd);%create a binary variable for each safety constraint
    fOld = f; %% f Old
    fNew = [zeros(1,totalSegments-1)';1;zeros(1,S)'];%function that we try to minimize which is exit time from CZ; the last S variables are augmented binary variables
    f = vertcat(f,fNew); % concatened f with fnew
    
    intconNew = (length(fOld)+totalSegments+1):1:length(f);%new binary variables
    intcon = [intcon, intconNew];% all binary variables
    constrNum = S*2+2*(totalSegments-1) + constrNum;
end

A = zeros(constrNum,length(f));
B = zeros(constrNum,1);

fNew = 0;
constrNumNew =0;
lb = [];
ub = [];
Aeq = [];
Beq =[];
%% Main loop to create the matrices for CPLEX
for ii = 1:totalVehicles
    fOld = fNew; % %total number
    constrNumOld = constrNumNew;%total number of constraint for the previous CAV(rows in A)
    PathNumber = path(ii);
    totalSegments = nnz(pathInfo(PathNumber,:));
    conflictInfoZoneInd  = ConflictInfo(ii).zoneInd;
    S = length(conflictInfoZoneInd);
    fNew = totalSegments + S + fOld ;
    constrNumNew = S*2+2*(totalSegments-1)+constrNumOld;
    
    for jj = 1:totalSegments-1
        %The followings are for X2-X1<D2
        A(constrNumOld + jj, jj + fOld) = -1;
        A(constrNumOld + jj, jj+1 + fOld) = +1;
        B(constrNumOld + jj) = Deadline(ii,jj+1);
        %The followings are for X1-X2<-R2
        A(constrNumOld + jj + totalSegments-1, jj + fOld) = 1;
        A(constrNumOld + jj + totalSegments-1, jj+1 + fOld) = -1;
        B(constrNumOld + jj + totalSegments-1) = - Release(ii,jj+1);
        
    end
    
    for jj = 1:S
        Var = conflictInfoZoneInd(jj)-1; %var is to determine which schedule of CAV $i$ is being dealt in the safety constraint --> recall that X1 = T2, since the first schedule is known and equal to
        varOther = ConflictInfo(ii).schedule(jj)-1;
        otherCAVID = ConflictInfo(ii).otherCAVID(jj);
        %the entry time
        %the following represents the constraint -T_i- B_i * M  + T_j < = - h
        %(equation (67) in the paper)
        A(constrNumOld + 2*(totalSegments-1)+jj, Var+fOld)= -1;
        A(constrNumOld + 2*(totalSegments-1)+jj, totalSegments+jj +fOld)= -M;
        
        %Find the variable corresponding to the schdule of CAV j
        VariablesBefore = 0;
        for kk = 1:otherCAVID-1
            PathNumberOther = path(kk);
            totalSegmentsOther = nnz(pathInfo(PathNumberOther,:));
            conflictInfoZoneIndOther  = ConflictInfo(kk).zoneInd;
            SOther = length(conflictInfoZoneIndOther);
            VariablesBefore = VariablesBefore + totalSegmentsOther + SOther; %to count total variables before the corresponding schedule
        end
        
        A(constrNumOld + 2*(totalSegments-1)+jj, VariablesBefore + varOther)= 1;
        
        B(constrNumOld + 2*(totalSegments-1)+jj)= -timeHeadway;
        
        %the following represents the constraint T_i- T_j + B_i * M < = M - h
        
        A(constrNumOld + 2*(totalSegments-1)+ S + jj, Var+fOld)= 1;
        A(constrNumOld + 2*(totalSegments-1)+ S + jj, totalSegments+jj + fOld)= M;
        A(constrNumOld + 2*(totalSegments-1)+ S + jj, VariablesBefore + varOther)= -1;
        
        B(constrNumOld + 2*(totalSegments-1)+ S + jj) = M -timeHeadway;
    end
    

    %%
    %%This is to define equality constraints for binary variables if they are
    %%related for zones that are after each other  We have Bi, i =1 ,...,s;
    %%In the matrix of random variables (f), binary variables start after the
    %%schedules + final time--> B1 is located at {totalSegments+1}
    
    AEQQ =zeros(1,length(f));
    BEQQ = [];
    numEqu = 0; %% this is the number of equalities for binary variables
    for jj = 1:S-1
        
        currentBinary = ConflictInfo(ii).rear(jj);
        if currentBinary == 0
            continue %% this is not eual to any other Bi, i= 1,...,s
        end
        if (currentBinary <0)
            %%CAV i is on the same path with the vechile k and cannot reach
            %%sooner than vehicle k --> B_i = 0;
            numEqu = numEqu + 1;
            AEQQ(numEqu,fOld + totalSegments+ jj) = 1;
            BEQQ(numEqu) = 0;
        end
        
        nextBinary = ConflictInfo(ii).rear(jj+1);
        
        if currentBinary == nextBinary
            %% we should add B_{i} - B_{i+1}  = 0
            numEqu = numEqu + 1;
            AEQQ(numEqu,fOld + totalSegments+ jj) = 1;
            AEQQ(numEqu,fOld + totalSegments+ jj+1) = -1;
            BEQQ(numEqu) = 0;
        end
    end
    if nnz(AEQQ)~=0
        Aeq = [Aeq; AEQQ];
        Beq = vertcat(Beq, BEQQ');
    end 
    
    lbNew=[Tinitial(ii)+Release(ii,1);zeros(totalSegments + S-1,1)];
    ubNew=[Tinitial(ii)+Deadline(ii,1);inf(totalSegments-1,1);ones(S,1)];%one is to force the variable to be binary

    lb = vertcat(lb,lbNew);
    ub = vertcat(ub,ubNew); 
end
fprintf("___________\n");


xTest = [15.8300000000000;16.8100000000000;17.7900000000000;33.6200000000000;17.8300000000000;18.8100000000000;24.8700000000000;25.8500000000000;26.8300000000000;42.6600000000000;0;40.8300000000000;41.8100000000000;42.7900000000000;58.6200000000000;0;0;0;0];

%%

if (solver == "IBMCPLEX")
    sostype =[];
    sosind = [];
    soswt  = [];
    ctype(1:numel(f))='C';
    ctype(intcon) ='B';
    tic;
    [x,fval,exitflag,output] = cplexmilp(f,A,B,Aeq,Beq,sostype,sosind,soswt,lb,ub,ctype);
    computationTime = toc;
    fprintf('==============\n');
    fprintf('%s .\n', output.message);
end
Schedule = x;
end

