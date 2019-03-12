function[a,b,c,d]= energyMatrix(t,tm,p,pm,v,vm)

    % This function allows finding a set of constants to control each vehicle 
    % on the road
    % t: current/initial time; 
    % p: current/initial position;
    % v: current/initial speed;
    
    % tm: final time, entry time to the merging zone (assuming constant speed within
    % merging zone)
    % pm: final position, start point of the merging zone
    % vm: final speed (assuming same speed to the entry speed of control
    % zone)

    A =[t^3/6 t^2/2 t 1; t^2/2 t 1 0; tm^3/6 tm^2/2 tm 1; tm^2/2 tm 1 0];
    Y =[p; v; pm; vm];

    % x=mldivide(A, Y) or x=A\Y solves the system of linear equations A*x=Y, by Matlab definition
    X = A\Y; 
    
%     A_aux=A_mtx'*A_mtx;
%     X_mtx=pinv(A_aux)*A_mtx'*Y_mtx;
        
     a=X(1);
     b=X(2);
     c=X(3);
     d=X(4);

end