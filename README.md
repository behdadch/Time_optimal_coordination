# SchedulingSimulation

Functions:

1) timeArray = randomTimeGen(number) 

2) conflictFinder(path,pathInfo,totalVehicles);

3) ReleaseDeadlineFinder(path,pathInfo,totalVehicles,z)

4) [pStart,pEnd,vStart,vEnd] = mapGeometry(vehicleIndex,zoneNumber,pathInfo,path,z)

5) R(i,j) = timeOptimal(vStart,vEnd,pStart,pEnd);

6) D(i,j) = deadline(vStart,vEnd,pStart,pEnd);

7)[duration,output] = scheduleFinderDec(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo)

8)[Schedule, computationTime, exitflag,output] = MILPDec(Tinitial,totalSegments,Release,Deadline,conflictInfoZoneInd,conflictInfoSchedule,conflictInfoRear,timeHeadway,varargin) 

9)[duration,output] = scheduleFinderCentralized(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo)

10) [Sol,computationTime,exitflag,output] = MILPCentralized(T(:,1), path, pathInfo, R(:,:), D(:,:), ConflictInfo, timeHeadway, totalVehicles);

11) [zone,index,finish]=zoneCheck2(vehicleIndex,time,pathInfo,path)

12)  mapBuilder()

13) [pos,vel,control,solved]=controller(i,j,type,pathInfo,time,path,solved,z)

14) [b,c]= timeMatrix(pStart,pEnd,vStart,vEnd,tEnter,tExit,Acc)

15) [a,b,c,d]= energyMatrix(t,tm,p,pm,v,vm)

16)[a,b,c,d,f,g,t1]= UlimMatrix(t_0,t_m,p_0,p_m,v_0,v_m,u_min)

16) F = ControlActive(x,t0,tm,p0,pm,v0,vm,U)

17) F = UmaxUnconUmin(x,t0,tm,p0,pm,v0,vm,U1,U2)

