# SchedulingSimulation
## Author:[Behdad Chalaki](https://sites.udel.edu/bchalaki/)
## Paper: Time-optimal coordination for connected and automated vehicles at adjacent intersections [pdf](https://doi.org/10.1109/TITS.2021.3123479)

The schduling problem in the upper-level planning relies on IBM CPLEX, which needs to be installed. The simulation has been tested for version **IBM CPLEX 12.10**, it may not work in other versions. 

```
@article{chalaki2021time,
  title={Time-optimal coordination for connected and automated vehicles at adjacent intersections},
  author={Chalaki, Behdad and Malikopoulos, Andreas A},
  journal={IEEE Transactions on Intelligent Transportation Systems},
  year={2021},
  publisher={IEEE}
}
```
### Functions

### 1. timeArray = randomTimeGen(number) 

*Create a random entry time for number vehicles based on poission distribution*

### 2. conflictFinder(path,pathInfo,totalVehicles)

  *This function will find the conflict set for the vehicle i with respect to all the vehicles with smaller indices (FCFS) and push it into the global variable conflict*
  
### 3. ReleaseDeadlineFinder(path,pathInfo,totalVehicles,z)

  *This function computes release time and deadline and push them in their global variable*

### 4. [pStart,pEnd,vStart,vEnd] = mapGeometry(vehicleIndex,zoneNumber,pathInfo,path,z)

"This function wil return the entry and exit info of zoneNumber for the vehicleIndex
"zoneNumber is the topological number of zone"

### 5. R(i,j) = timeOptimal(vStart,vEnd,pStart,pEnd);

### 6. D(i,j) = deadline(vStart,vEnd,pStart,pEnd);

### 7. [duration,output] = scheduleFinderDec(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo)


### 8. [Schedule, computationTime, exitflag,output] = MILPDec(Tinitial,totalSegments,Release,Deadline,conflictInfoZoneInd,conflictInfoSchedule,conflictInfoRear,timeHeadway,varargin) 


### 9. [duration,output] = scheduleFinderCentralized(path,pathInfo,totalVehicles,timeHeadway,TZeros,zoneInfo)


### 10. [Sol,computationTime,exitflag,output] = MILPCentralized(T(:,1), path, pathInfo, R(:,:), D(:,:), ConflictInfo, timeHeadway, totalVehicles);


### 11. [zone,index,finish]=zoneCheck(vehicleIndex,time,pathInfo,path)

"This function will find zone that CAV is travelling through from the schedule of the CAV and current time"

### 12.  mapBuilder()

"Build environment geometry for Animation"

### 13. [pos,vel,control,solved]=controller(i,j,type,pathInfo,time,path,solved,z)


### 14. [b,c]= timeMatrix(pStart,pEnd,vStart,vEnd,tEnter,tExit,Acc)


### 15. [a,b,c,d]= energyMatrix(t,tm,p,pm,v,vm)


### 16. [a,b,c,d,f,g,t1]= UlimMatrix(t_0,t_m,p_0,p_m,v_0,v_m,u_min)


### 17. F = ControlActive(x,t0,tm,p0,pm,v0,vm,U)


### 18. F = UmaxUnconUmin(x,t0,tm,p0,pm,v0,vm,U1,U2)

