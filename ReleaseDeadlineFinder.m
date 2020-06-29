function ReleaseDeadlineFinder(path,pathInfo,totalVehicles,z)
global R
global D
R =[];
D =[];
for i=1:totalVehicles
    PathNumber = path(i);
    j=1;
    while j <= nnz(pathInfo(PathNumber,:))
        m = pathInfo(PathNumber,j);
        [pStart,pEnd,vStart,vEnd] = mapGeometry(i,m,pathInfo,path,z);
        R(i,j) = timeOptimal(vStart,vEnd,pStart,pEnd);
        D(i,j) = deadline(vStart,vEnd,pStart,pEnd);
        j = j+1;
    end
end
end
