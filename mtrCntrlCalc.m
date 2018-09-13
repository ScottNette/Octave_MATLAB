clc
clear

adcCounts_max = 1023;
updateRate = 1/3000;

mtrSpd_RPM = 1000 ;%[0:1:500];
mtrSpd_rps = mtrSpd_RPM/60;

updateCounts = (1/updateRate)./mtrSpd_rps;


figure(1),clf
plot(mtrSpd_RPM, updateCounts)
xlabel('Speed (RPM)')
ylabel('Num updates/rev')


% 6 samples @ 10,000 RPM
updateSectors = updateCounts(end)
updateSectorSize_deg = 360/updateSectors % deg
updateSectorSize_cnts = floor(1023/updateSectors)
