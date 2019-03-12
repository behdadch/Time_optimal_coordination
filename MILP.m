function y = MILP(temp,earliestEnter,timeHeadway)
M = 100;
size = length(temp);
f = [1,zeros(1,size)];
intcon = 2:1:(size+1);
A = zeros(size*2,length(f));
for i = 1:size
    A(i,i+1) = -M;
    A(i+size,i+1) = M;
    A(i,1) = -1;
    A(i+size,1) = 1; 
end 
b = zeros(size*2,1);
for i = 1:size
    b(i,1) = -timeHeadway-temp(i);
    b(i+size,1) = -timeHeadway+temp(i)+M;
end 
Aeq=[];
Beq=[];
lb=[earliestEnter,zeros(1,size)];
ub=[inf,ones(1,size)];
x = intlinprog(f,intcon,A,b,Aeq,Beq,lb,ub);
y=x(1);
end

