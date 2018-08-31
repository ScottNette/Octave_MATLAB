clear
clc

potMtrCntsPrev = 0;
potMtrCnts = 0;
mtrPosCnts = 0;


realMrPos = 10;
realMrPosPrev = 10;

realMrrate = 0;

sampledMtrPos = 0;
wrapOffset = 0;
diffCounts = 0;

diff_cnts = 0;

meas_spd =0;
countIdx = 1;

deadzone_hard = 4; % deg2
deadzone_soft_p = 360 - deadzone_hard - deadzone_hard/2; %deg
deadzone_soft_n = deadzone_hard + deadzone_hard/2; %deg

deadzoneSizeDeg = 360 -deadzone_soft_p + deadzone_soft_n;
extCnt2Deg = (360-15)/1023;
extDeg2Cnt = 1/extCnt2Deg;
deadzoneSizeCnt = floor(deadzoneSizeDeg*extDeg2Cnt);
totRevCnt = 1023 + floor(deadzoneSizeCnt);

dt_sim = 1/100000;
dt_cntrl = 1/1000;
t = [0:dt_sim:1];

tao = 0.0000000001;
alpha = dt_cntrl/(tao + dt_cntrl);



mtrSpeed_rpm = (300*sin(2*pi*100*t)+3000).* min(exp(t*5)-1,1) ;
mtrSpeed_rps = mtrSpeed_rpm/60;
mtrSpeed_dps = mtrSpeed_rps *360;
mtrSpeed_cps = mtrSpeed_rps*1023;

startOffset_deg = 10;

WRAP_THRESHOLD = 100;


numSectors = 12;
sizeSector = ceil(1023/numSectors);
prevSector = 0;
curSector = 0;
secVect = [0:numSectors-1];
predSector = 0;

deadzoneEN = 1;
wrapEn     = 1;

max_RPM_sat = 10000000;


predictMode = 0;
predictModePrev = 0;

% startOffset_deg/360
% curSector      = 
% prevSector     = curSector;
% sampledMtrPos  = 
% potMtrCntsPrev = sampledMtrPos;


for ii = 1:length(t)
  realMrPos = realMrPos + mtrSpeed_dps(ii)*dt_sim;
  
     mtrPosCnts = realMrPos*1023/360;
  
  realMrrate(ii) = (realMrPos - realMrPosPrev)/(dt_sim);
  realMrPosPrev = realMrPos;
  
  realMrPos_deg(ii) = realMrPos;
  realMtrPos_cnts(ii) = mtrPosCnts;
  
  if (mod(ii,dt_cntrl/dt_sim) == 0)
      mtrDir = sign(mtrSpeed_cps(ii));
  
    %% Deadzone
    if (deadzoneEN)
      deadzoneCalc = mod(realMrPos,360);
      if (((360 -deadzone_hard/2) < deadzoneCalc ) || (deadzoneCalc < deadzone_hard/2))
         sampledMtrPos = randi([0 1023]);
          sampledMtrPos = 0;
%         disp('rand')
      elseif (deadzoneCalc > deadzone_soft_p)
        sampledMtrPos = 1023;
%                 disp('hold high')
      elseif(deadzoneCalc < deadzone_soft_n)
      % do neg
      sampledMtrPos = 0;
%               disp('hold low')
      else
        sampledMtrPos = mod(floor(mtrPosCnts),1023) ;
      end
      
      
    else
      sampledMtrPos = mod(floor(mtrPosCnts),1023) ;
    end
    
    potMtrCnts = sampledMtrPos;   
    
    
    %%
    
    if (wrapEn )
    
     % Find out where we area
       curSector = ceil(sampledMtrPos / sizeSector);
       
       if((sampledMtrPos == 0) || (sampledMtrPos == 1023))
          predictMode = 1;
       end
       
         if (predictModePrev)
          predictZone = deadzoneSizeCnt;
          
       else
          predictZone = 0;
       end
       
       
       
              
       
             % Wrap check down
      if( (curSector == 1) && ((prevSector == 13) || (prevSector == 12 )))
          wrapOffset = 1023 + predictZone- potMtrCntsPrev;
          diffCounts = (potMtrCnts + wrapOffset);
      % Wrap check up  
       elseif( ((curSector == 12) || (curSector == 13 )) && (prevSector = 0))
           wrapOffset = 1023+predictZone - potMtrCnts;
           diffCounts = -(wrapOffset + potMtrCntsPrev);
           
       else
       
         if ((abs(curSector - prevSector) > 1) )
            predictMode = 1;
         end
         
    
         
         if(predictMode)
            sampledMtrPos = mod(potMtrCntsPrev + diffCounts, 1023+deadzoneSizeCnt);
            curSector = ceil(sampledMtrPos / sizeSector);
             predictZone = deadzoneSizeCnt;
             predictMode = 0;
         end
         
 

     
          tempDiff = sampledMtrPos - potMtrCntsPrev;
          diffCounts = tempDiff;
          
      end    
          
  
 end

      
%    if ( (meas_, spd(countIdx) < 0)  ||   (meas_spd(countIdx) > 4000))
if ( abs(diffCounts) > 35)
       disp('break')
end
    

    meas_predSect(countIdx)= predSector;
    meas_dir(countIdx)  = mtrDir;
    meas_sct(countIdx) = curSector;
    meas_cnt(countIdx) = sampledMtrPos; 
    diff_cnts(countIdx) = diffCounts;
    meas_spd(countIdx) = diffCounts*(60/1023)/(dt_cntrl);
    meas_spd(countIdx) = min(meas_spd(countIdx),max_RPM_sat);
    meas_spd(countIdx) = max(meas_spd(countIdx),-max_RPM_sat);
    real_spd(countIdx)  = mtrSpeed_rpm(ii);
    meas_time(countIdx) = t(ii); 
    
    
    

    
    predictModePrev = predictMode;
    potMtrCntsPrev = sampledMtrPos;
    prevSector = curSector;
    countIdx = countIdx+1;
  end
  
  
end

% meas_spd(2) = mtrSpeed_rpm(2);

figure(1),clf
hold all
%plot(t,realMrrate)
plot(t, mtrSpeed_rpm)
plot(meas_time, meas_spd,'-x')

xlabel('Time')
ylabel('RPM')


figure(2),clf
hold all
plot(meas_time,meas_cnt,'x')
% plot(t, realMtrPos_cnts*360/1023)
% plot(t, realMrPos_deg)

