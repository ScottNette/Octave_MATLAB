clear
clc

potMtrCntsPrev = 0;
potMtrCnts = 0;
mtrPos = 0;
realMrPos = 0;
realMrPosPrev = 0;


sampledMtrPos = 0;
wrapOffset = 0;
diffCounts = 0;


mtrSpeed_rpm = 10000;
mtrSpeed_rps = mtrSpeed_rpm/60;
mtrSpeed_cps = mtrSpeed_rps*1023;

meas_spd =0;
countIdx = 1;

dt_sim = 1/10000;
dt_cntrl = 1/1000;
t = [0:dt_sim:1];

WRAP_THRESHOLD = 200;


for ii = 1:length(t)
  mtrPos = mtrPos + mtrSpeed_cps*dt_sim;
  
  realMrPos = realMrPos + mtrSpeed_rpm*dt_sim;
  realMrrate = (realMrPos - realMrPosPrev)/(dt_sim);
  realMrPosPrev = realMrPos;
  
  if (mod(t(ii),dt_cntrl) == 0)
    sampledMtrPos = mod(floor(mtrPos),1023) ;
    tempDiff = sampledMtrPos - potMtrCntsPrev;
    
    potMtrCnts = sampledMtrPos;

    %/*  Wrap finder */
      if (tempDiff > WRAP_THRESHOLD)
        % forward Wrap, 800 - 100 = 700 > 200
        wrapOffset = 1023 - potMtrCnts;
        diffCounts = -(wrapOffset + potMtrCntsPrev);
      
      elseif(tempDiff < -WRAP_THRESHOLD)
        % Backwards, 60 - 900 = -840 < 200
        wrapOffset = 1023 - potMtrCntsPrev;
        diffCounts = (potMtrCnts + wrapOffset);
      
      else
      % no motion
      wrapOffset = 0;
      diffCounts = tempDiff;
    end
    
    diff_cnts(countIdx) = diffCounts;
    meas_spd(countIdx) = diffCounts*(60/1023)/(1/1000);
    real_spd(countIdx)  = mtrSpeed_rpm;
    meas_time(countIdx) = t(ii);
    countIdx = countIdx+1;
    
    potMtrCntsPrev = sampledMtrPos;
  end
  
  
end

figure(1),clf
hold all
plot(meas_time, real_spd)
plot(meas_time, meas_spd)









