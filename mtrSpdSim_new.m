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

deadzone_hard = 4; % degs
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


mtrSpeed_rpm = (200*sin(2*pi*10*t)).* min(exp(t*5)-1,1) ;
mtrSpeed_rps = mtrSpeed_rpm/60;
mtrSpeed_dps = mtrSpeed_rps *360;
mtrSpeed_cps = mtrSpeed_rps*1023;

startOffset_deg = 10;

WRAP_THRESHOLD = 20;

numSectors = 12;
sizeSector = ceil(1023/numSectors);
prevSector = ceil(realMrPos / sizeSector);
curSector = prevSector;
secVect = [0:numSectors-1];
predSector = 0;

deadzoneEN = 1;
wrapEn     = 1;  % 1 = wrap logic, 0 = observer

max_RPM_sat = 1000;

initRun = 0;
predictMode = 0;
predictModePrev = 0;
diffCountsPrev =  mod(realMrPos,360);

% startOffset_deg/360
% curSector      = 
% prevSector     = curSector;
% sampledMtrPos  = 
% potMtrCntsPrev = sampledMtrPos;

 
 wrapCounter = 0;
 trackingAckPrev = 0;
      
      x0 = zeros(4);
      x = x0;
%%      
wrapCnt = 0;
trackingAck = 0;

for ii = 1:length(t)
  realMrPos = realMrPos + mtrSpeed_dps(ii)*dt_sim;
  
     mtrPosCnts = realMrPos*1023/360;
  
  realMrrate(ii) = (realMrPos - realMrPosPrev)/(dt_sim);
  realMrPosPrev = realMrPos;
  
  realMrPos_deg(ii) = realMrPos;
  realMtrPos_cnts(ii) = mtrPosCnts;
  
%   if (t(ii) >= 0.103)
%      disp('time hit') 
%   end
  
  
  if (mod(ii,dt_cntrl/dt_sim) == 0)
    
    
      mtrDir = sign(mtrSpeed_cps(ii));
  
    %% Deadzone
    if (deadzoneEN)
      deadzoneCalc = mod(realMrPos,360);
      deadzoneFlag = 1;
      if (((360 -deadzone_hard/2) < deadzoneCalc ) || (deadzoneCalc < deadzone_hard/2))
        
         sampledMtrPos = randi([0 1023]);
%           sampledMtrPos = 0;
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
        deadzoneFlag = 0;
      end
      
      
    else
      sampledMtrPos = mod(floor(mtrPosCnts),1023) ;
    end
    
    potMtrCnts = sampledMtrPos  
   
    %% end zone
   
      
      
    
    
    
    if(initRun ==  0)
      potMtrCntsPrev = sampledMtrPos;
      diffCountsPrev = sampledMtrPos;
    end
    
    
    
    
    if (wrapEn )
    
     % Find out where we area
       curSector = ceil(sampledMtrPos / sizeSector);
       
       if((sampledMtrPos == 0) || (sampledMtrPos == 1023))
          predictMode = 1;
          trackingAck = 0;
          trackingAckPrev = 0; 
       else
           if (abs(sampledMtrPos - trackingAckPrev) < WRAP_THRESHOLD )
               trackingAckPrev = sampledMtrPos;
               trackingAck = trackingAck +1;
           else
               trackingAck = 0;
           end
           trackingAckPrev = sampledMtrPos;
       end
       
       if (predictModePrev)
          predictZone = 0; %deadzoneSizeCnt;
       else
          predictZone = 0;
       end
       

        
         
        % Wrap check down
      if( ((curSector == 1) || (curSector == 0)) && ((prevSector == 13) || (prevSector == 12 )))
        wrapOffset = 1023 -  potMtrCntsPrev;
        diffCounts = (sampledMtrPos + wrapOffset);
        wrapCnt = wrapCnt +1;
      % Wrap check up  
      elseif( ((curSector == 12) || (curSector == 13 )) && ((prevSector == 0) ||  (prevSector == 1)))
        wrapOffset = 1023 -  sampledMtrPos;
        diffCounts = -(wrapOffset + potMtrCntsPrev);
        wrapCnt = wrapCnt - 1;
      else
          
        if (abs(curSector - prevSector) > 1)
          predictMode = 1;
%         elseif(((curSector - prevSector) ~= sign(mtrDir)) && ((curSector - prevSector) ~= 0))
%          predictMode = 1;
%         elseif (( abs(sampledMtrPos - potMtrCntsPrev) > WRAP_THRESHOLD) && (diffCountsPrev ~= 0)) % add accel limit
%          predictMode = 1;
        end
           
        

            if(predictMode)
                 if ((trackingAck) >= 2)
                    predictMode = 0;
                 else

                  sampledMtrPos = mod(potMtrCntsPrev + diffCountsPrev, 1023);
                  curSector = ceil(sampledMtrPos / sizeSector);
                  predictZone = 0;

                        % Wrap check down
                      if( ((curSector == 1) || (curSector == 0)) && ((prevSector == 13) || (prevSector == 12 )))
                        wrapOffset = 1023 -  potMtrCntsPrev;
                        diffCounts = (sampledMtrPos + wrapOffset);
                        wrapCnt = wrapCnt +1;
                      % Wrap check up  
                      elseif( ((curSector == 12) || (curSector == 13 )) && ((prevSector == 0) ||  (prevSector == 1)))
                        wrapOffset = 1023 -  sampledMtrPos;
                        diffCounts = -(wrapOffset + potMtrCntsPrev);
                        wrapCnt = wrapCnt - 1;
                      end
                 end
            end

                tempDiff = sampledMtrPos - potMtrCntsPrev;
                diffCounts = tempDiff;

                diffCounts = min(diffCounts, WRAP_THRESHOLD);
%                 diffCounts = min(diffCounts, WRAP_THRESHOLD);

                mtrPosCntsUnwrapped = sampledMtrPos + wrapCnt*1023;

            
        mtrPosCntsUnwrapped = sampledMtrPos + wrapCnt*1023;


      end
    
         
         
%         else
%         
%       
%         tempDiff = sampledMtrPos - potMtrCntsPrev;
%         diffCounts = tempDiff;
%         
%         diffCounts = min(diffCounts, WRAP_THRESHOLD);
%         
%         mtrPosCntsUnwrapped = sampledMtrPos + wrapCnt*1023;
%         
%        
%       
%         
%      
%         calcMtrPos = (sampledMtrPos+wrapCnt*1023);
%       
%        tempDiff = (sampledMtrPos+wrapCnt*1023) - (potMtrCntsPrev+wrapCnt*1023);
%         
% %         y = C*x; % + D*tempDiff;
%         x = A*x; % + B*tempDiff;
        
  %%      diffCounts = y(1);
        
%         est_out = y(4);
        
     
      
    else
      
      %% original logic
      tempDiff = sampledMtrPos - potMtrCntsPrev;
      diffCounts = tempDiff;
      
    
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
 %       no motion
        wrapOffset = 0;
        diffCounts = tempDiff;
      end
  
    end

      
    %    if ( (meas_, spd(countIdx) < 0)  ||   (meas_spd(countIdx) > 4000))
   if ( (diffCounts) < -30 )
        disp('break')
   end

%    if (   trackingAck == 0)
%          disp('break')
%     end
      

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
    meas_mtrPos(countIdx)  = realMrPos;
    
    meas_potMtrCnts(countIdx) = potMtrCnts;
    meas_deadzoneFlag(countIdx) = deadzoneFlag;   
    meas_trackingAck(countIdx) =trackingAck;
    
    meas_mtrPosCntsUnwrapped(countIdx)  = mtrPosCntsUnwrapped;
    
    meas_mtrPosCnts(countIdx)  = mod(mtrPosCnts,1023);
    
%     meas_calcMtrPos(countIdx)  = calcMtrPos;
    meas_wrapCnt(countIdx)  = wrapCnt;

    meas_pred(countIdx) = predictMode;     
%     meas_predPosCnts(countIdx) = predPosCnts;    
%     meas_estm(countIdx) = est_out*(60/1023)/(dt_cntrl);     
    
    predictModePrev = predictMode;
    diffCountsPrev = diffCounts;
    potMtrCntsPrev = sampledMtrPos;
    prevSector = curSector;
    countIdx = countIdx+1;
    
    predictMode = 0;
    
    initRun = 1;
  end
  
  
end

% meas_spd(2) = mtrSpeed_rpm(2);

figure(1),clf
% subplot(211)
hold all
%plot(t,realMrrate)
plot(t, mtrSpeed_rpm)
plot(meas_time, meas_spd,'-x')
% plot(meas_time, meas_estm,'-o')

% plot(plot(meas_time, meas_estm)
plot(meas_time,meas_pred*300,'o')
plot(meas_time,meas_potMtrCnts*10)
plot(meas_time,meas_trackingAck*100)
plot(meas_time,meas_deadzoneFlag*1000, 'kx')

legend({'Actual'; 'wrap'; 'predict mode'; 'potCnts'; 'tracking'})

xlabel('Time')
ylabel('RPM')


figure(2),clf
hold all
plot(meas_time,meas_mtrPosCntsUnwrapped*360/1023)
plot(meas_time,meas_mtrPos,'x')
% plot(meas_time,meas_pred*3000,'x')
legend({'unwrap cnts';'real'})
figure(3),clf
plot(meas_time,diff_cnts)
%plot(meas_time, meas_estm,'-o')
% plot(t, realMtrPos_cnts*360/1023)
% plot(t, realMrPos_deg)

