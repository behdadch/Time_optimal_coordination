function Schedule = MILPFinal(Tinitial,totalSegments,Release,Deadline,ConflictInfoOrder,ConflictInfoSchedule,timeHeadway)
%shcedule is a vector;
M = 100;
S = length(ConflictInfoOrder);%create a binary variable for each safety constraint
f = [zeros(1,totalSegments-1)';1;zeros(1,S)'];%function that we try to minimize which is exit time from CZ
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
    Var = ConflictInfoOrder(i)-1; %Variable number for the zone since X1=T2 and T1 is already known
    A(2*(totalSegments-1)+i,Var)= -1;
    A(2*(totalSegments-1)+i,totalSegments+i)= -M;
    B(2*(totalSegments-1)+i)= -ConflictInfoSchedule(i)-timeHeadway;
    A(2*(totalSegments-1)+S+i,Var)= 1;
    A(2*(totalSegments-1)+S+i,totalSegments+i)= M;
    B(2*(totalSegments-1)+S+i) = M + ConflictInfoSchedule(i)-timeHeadway;
end 
Aeq =[];
Beq =[];
lb=[Tinitial+Release(1);zeros(length(f)-1,1)];
ub=[Tinitial+Deadline(1);inf(totalSegments-1,1);ones(S,1)];%one is to force the variable to be binary
x = intlinprog(f,intcon,A,B,Aeq,Beq,lb,ub);
Schedule = x(1:totalSegments)';
end

