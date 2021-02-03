function timeArray = randomTimeGen(number,seed)
time_step = 0.1;
tt =105;
finalRelease = 100; %no vehicle be release after this
while(max(tt)> finalRelease) %%
mu=1;%mean of the random number %the rate for the poisson distribution
expRnd = exprnd(mu,1,number); % generate an array (1Xnumber) of random number from exp distribution with 
tt = cumsum(expRnd); %%this is poisson distribution with rate mu
max(tt);
tnew = ceil(tt/time_step);
t = tnew*time_step;
end
timeArray = t(1:number);
end