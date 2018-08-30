clear
clc

potMtrCntsPrev = 0;
potMtrCnts = 0;
mtrPos = 0;
realMrPos = 0;
realMrPosPrev = 0;

realMrrate = 0;

sampledMtrPos = 0;
wrapOffset = 0;
diffCounts = 0;

diff_cnts = 0;

meas_spd =0;
countIdx = 1;

deadzone_hard = 8; % deg2
deadzone_soft_p = 360 - deadzone_hard - deadzone_hard/2; %deg
deadzone_soft_n = deadzone_hard + deadzone_hard/2; %deg




dt_sim = 1/100000;
dt_cntrl = 1/1000;
t = [0:dt_sim:1];

tao = 0;
alpha = dt_cntrl/(tao + dt_cntrl);



mtrSpeed_rpm = 5*sin(2*pi*1*t)+500;
mtrSpeed_rps = mtrSpeed_rpm/60;
mtrSpeed_dps = mtrSpeed_rps *360;
mtrSpeed_cps = mtrSpeed_rps*1023;

WRAP_THRESHOLD = 200;


numSectors = 12;
sizeSector = ceil(1023/12);
prevSector = 0;
curSector = 0;


deadzoneEN = 1;
wrapEn     = 1;

max_RPM_sat = 10000;

for ii = 1:length(t)
  mtrPos = mtrPos + mtrSpeed_cps(ii)*dt_sim;
  realMrPos = realMrPos + mtrSpeed_dps(ii)*dt_sim;
  
  realMrrate(ii) = (realMrPos - realMrPosPrev)/(dt_sim);
  realMrPosPrev = realMrPos;
  
  realMrPos_deg(ii) = realMrPos;
  
  if (mod(t(ii),dt_cntrl) == 0)
    

      mtrDir = sign(mtrSpeed_cps(ii));
  
    
    
    % Deadzone
    if (deadzoneEN)
      deadzoneCalc = mod(realMrPos,360);
      if (((360 -deadzone_hard/2) < deadzoneCalc ) || (deadzoneCalc < deadzone_hard/2))
         sampledMtrPos = randi([0 1023]);
##         sampledMtrPos = 0;
      elseif (deadzoneCalc > deadzone_soft_p)
        sampledMtrPos = 0;
      elseif(deadzoneCalc < deadzone_soft_n)
      % do neg
      sampledMtrPos = 0;
      else
        sampledMtrPos = mod(floor(mtrPos),1023) ;
      end
    else
      sampledMtrPos = mod(floor(mtrPos),1023) ;
    end
    
    
    
    if (wrapEn & (countIdx >2) )
      curSector = floor(sampledMtrPos / sizeSector);
      if ( (mtrDir*(curSector - prevSector) > 1)  || (mtrDir*(curSector - prevSector) < 0))
##          disp('wrapped')
##          disp(diffCounts)
         sampledMtrPos = mod(potMtrCntsPrev + mtrDir*mean(diff_cnts(end-10:end)),1023); % diffCounts,1023);
      end

    end
       
    
         
   % sampledMtrPos = potMtrCntsPrev + alpha*(sampledMtrPos - potMtrCntsPrev);
    
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
    
    meas_dir(countIdx)  = mtrDir;
    meas_sct(countIdx) = curSector;
    meas_cnt(countIdx) = sampledMtrPos; 
    diff_cnts(countIdx) = diffCounts;
    meas_spd(countIdx) = diffCounts*(60/1023)/(dt_cntrl);
    meas_spd(countIdx) = min(meas_spd(countIdx),max_RPM_sat);
    meas_spd(countIdx) = max(meas_spd(countIdx),-max_RPM_sat);
    real_spd(countIdx)  = mtrSpeed_rpm(ii);
    meas_time(countIdx) = t(ii);
    
    
    
    if ( (meas_spd(countIdx) < 0)  ||   (meas_spd(countIdx) > 4000))
##      disp('break')
    end
    
    
    potMtrCntsPrev = sampledMtrPos;
    
    
    prevSector = curSector;
    countIdx = countIdx+1;
  end
  
  
end

 meas_spd(2) = mtrSpeed_rpm(2);

figure(1),clf
hold all
##plot(t,realMrrate)
plot(t, mtrSpeed_rpm)
plot(meas_time, meas_spd,'-x')

xlabel('Time')
ylabel('RPM')









