function [Schedule, computationTime, exitflag,output] = MILPDec(Tinitial,totalSegments,Release,Deadline,conflictInfoZoneInd,conflictInfoSchedule,conflictInfoRear,timeHeadway,varargin)
global FIFO
switch nargin
    case 9
        cav_id = varargin{1};
    otherwise
        cav_id = nan;
end
solver = "IBMCPLEX";
options = optimoptions('intlinprog','Display','off');
%shcedule is a vector;
M = 5000;% this is a big number for the big M method to handle disjunctive (OR) constraints
S = length(conflictInfoZoneInd);%create a binary variable for each safety constraint
f = [zeros(1,totalSegments-1)';1;zeros(1,S)'];%function that we try to minimize which is exit time from CZ; the last S variables are augmented binary variables
intcon = totalSegments+1:1:length(f);%all binary variables
A = zeros(S*2+2*(totalSegments-1),length(f));
B = zeros(S*2+2*(totalSegments-1),1);

for i = 1:totalSegments-1
    %The followings are for X2-X1<D2
    A(i,i) = -1;
    A(i,i+1) = +1;
    B(i) = Deadline(i+1);
    %The followings are for X1-X2<-R2
    A(i+totalSegments-1,i) = 1;
    A(i+totalSegments-1,i+1) = -1;
    B(i+totalSegments-1) = -Release(i+1);
end
for i = 1:S
    Var = conflictInfoZoneInd(i)-1; %var is to determine which schedule is being dealt in the safety constraint --> recall that X1 = T2, since the first schedule is known and equal to
    %the entry time
    %the following represents the constraint -T_i- B_i * M < = - h - T_j
    %(equation (67) in the paper)
    A(2*(totalSegments-1)+i,Var)= -1;
    A(2*(totalSegments-1)+i,totalSegments+i)= -M;
    B(2*(totalSegments-1)+i)= -conflictInfoSchedule(i)-timeHeadway;
    
    
    A(2*(totalSegments-1)+S+i,Var)= 1;
    A(2*(totalSegments-1)+S+i,totalSegments+i)= M;
    B(2*(totalSegments-1)+S+i) = M + conflictInfoSchedule(i)-timeHeadway;
end

Aeq =[];
Beq =[];
%%
%%This is to define equality constraints for binary variables if they are
%%related for zones that are after each other  We have Bi, i =1 ,...,s;
%%In the matrix of random variables (f), binary variables start after the
%%schedules + final time--> B1 is located at {totalSegments+1}
AEQQ =zeros(1,length(f));
BEQQ = [];
numEqu = 0; %% this is the number of equalities for binary variables
if ~FIFO
    for ii = 1:S-1
        currentBinary = conflictInfoRear(ii);
        if currentBinary == 0
            continue %% this is not eual to any other Bi, i= 1,...,s
        end
        if (currentBinary <0)
            %%CAV i is on the same path with the vechile k and cannot reach
            %%sooner than vehicle k --> B_i = 0;
            numEqu = numEqu + 1;
            AEQQ(numEqu,totalSegments+ ii) = 1;
            BEQQ(numEqu) = 0;
        end
        
        nextBinary = conflictInfoRear(ii+1);
        
        if currentBinary == nextBinary
            %% we should add B_{i} - B_{i+1}  = 0
            numEqu = numEqu + 1;
            AEQQ(numEqu,totalSegments+ ii) = 1;
            AEQQ(numEqu,totalSegments+ ii+1) = -1;
            BEQQ(numEqu) = 0;
        end
        
    end
else
    %Set all the binary variables to zero 
    %fprintf("Following FIFO Structure\n")
    for ii = 1:S
            AEQQ(ii,totalSegments+ ii) = 1;
            BEQQ(ii) = 0;
    end 
    
end

Aeq = AEQQ;
Beq = BEQQ';



%%
lb=[Tinitial+Release(1);zeros(length(f)-1,1)];
ub=[Tinitial+Deadline(1);inf(totalSegments-1,1);ones(S,1)];%one is to force the variable to be binary

%A
%B

if (solver == "IBMCPLEX")
    
    sostype =[];
    sosind = [];
    soswt  = [];
    ctype(1:numel(f))='C';
    ctype(intcon) ='B';
    tic;
    [x,fval,exitflag,output] = cplexmilp(f,A,B,Aeq,Beq,sostype,sosind,soswt,lb,ub,ctype);
    computationTime = toc;
    
    %fprintf('==============\n');
    %fprintf('%s for CAV %d.\n', output.message,cav_id);
else
    tic
    [x,fval,exitflag,output] = intlinprog(f,intcon,A,B,Aeq,Beq,lb,ub,[],options);
    computationTime = toc;
    fprintf('tfinal is : %4.2f', fval);
end
Schedule = x;
end

