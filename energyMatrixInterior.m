function X = energyMatrixInterior(t,p,v)
%speed and position is fixed at boundary
    
    totalZone = length(t)-1;%Total number of arcs
    n = totalZone*4 + (totalZone-1)*2; % 4 constant for each zone and 2 pi for each interior constraint ( 1 speed + 1 position );    
     
    A = zeros(n,n);
    Y = zeros(n,1);
    
    %X = [constants for the first arc, constant for the second arc, ..., constants for the last arc, (Pis first and second arc),..., (Pi last-1 and last arc)]
    %p and v at each time instant ( last time u is ==0)
    for i = 1:length(t) %2*length t  e.g. length = 5 -> 10 BC
        
        if i == length(t)
            A(2*i-1,4*i-7:4*i-4) = [t(i)^3/6 t(i)^2/2 t(i) 1];
            Y(2*i-1) = p(i);
            %A(2*i,4*i-7:4*i-4) = [t(i) 1 0 0]; %acceleration zero at final time 
            %Y(2*i) = 0;
            A(2*i,4*i-7:4*i-4) = [t(i)^2/2 t(i) 1 0];% speed at end
            Y(2*i) = v(i);
        else 
            A(2*i-1,4*i-3:4*i) = [t(i)^3/6 t(i)^2/2 t(i) 1]; % position at each time
            Y(2*i-1) = p(i);
            A(2*i,4*i-3:4*i) = [t(i)^2/2 t(i) 1 0];% speed at each time
            Y(2*i) = v(i);
        end
    end
    
    for i = 1:length(t)-2 % 2 *(length(t)-2) e.g. length = 5 -> 6 continuity 
        A(length(t)*2+ 2*i-1,4*i-3:4*i+4) = [t(i+1)^3/6 t(i+1)^2/2 t(i+1) 1 -t(i+1)^3/6 -t(i+1)^2/2 -t(i+1) -1];
        Y(length(t)*2+ 2*i-1)=0;
        A(length(t)*2+ 2*i,4*i-3:4*i+4)= [t(i+1)^2/2 t(i+1) 1 0 -t(i+1)^2/2 -t(i+1) -1 0];
        Y(length(t)*2+ 2*i)=0;
    end 
    
    cond = length(t)*2 + (length(t)-2)*2;

    for i = 1:length(t)-2 %e.g. 2 *(length(t)-2) e.g length = 5 -> 6 jump conditions
        %lambda p
        A(cond + 2*i-1,4*i-3) = 1;
        A(cond + 2*i-1,4*i+1) = 1;
        A(cond + 2*i-1,(length(t)-1)*4 + 2*i-1) = -1;
        Y(cond + 2*i-1) = 0;
        
        %lambda v
        A(cond + 2*i,4*i-3:4*i+4) = [-t(i+1) -1 0 0,t(i+1) 1 0 0];
        A(cond + 2*i,(length(t)-1)*4 + 2*i) = -1;
        Y(cond + 2*i) = 0;
    end 
    rank(A)
    % x=mldivide(A, Y) or x=A\Y solves the system of linear equations A*x=Y, by Matlab definition
    X = A\Y; 

end