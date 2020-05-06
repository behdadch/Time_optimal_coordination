function B = PrintPosition(v1,v2)
global conflict
a = conflict(v2,v1,:);
b = reshape(a,1,[]);
B = b(b~=0);

end