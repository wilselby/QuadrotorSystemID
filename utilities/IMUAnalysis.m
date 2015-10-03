% addpath('~/carmen3D/src/matlab:', ...
%     '~/carmen3D/bin:', ...
%     '~/carmen3D/src/logger:', ...
%     '~/carmen3D/sysidAnalysis/rls/matlab/:', ...
%     '~/carmen3D/sysidAnalysis/angleStuff/:');
% 
% 


% rawViconData = d.VICON_yellowpelican;
rawViconData = d.VICON_MagQuad;
viconTimes = rawViconData(:,end);
viconrpy = bot_quat2rpy(rawViconData(:,5:8));
viconData = [rawViconData(:,2:4),viconrpy(:,[3,2,1])];


bodyStates = convertGlobalToBody(viconData);
viconPer = median(diff(viconTimes));

%% Differentiate & Filter

% tbd - decide between meanFilter & smooth
gxv = meanFilter(diff(viconData(:,1)),20)./viconPer;
gxa = meanFilter(diff(gxv),20)./viconPer;
% gxv = smooth(diff(viconData(:,1)),40)./viconPer;
% gxa = smooth(diff(gxv),40)./viconPer;
gxv = [0; gxv];
gxa = [0; 0; gxa];
gxa([1:40,end-40:end])=0; %get rid of end effects...

gyv = meanFilter(diff(viconData(:,2)),20)./viconPer;
gya = meanFilter(diff(gyv),20)./viconPer;
% gyv = smooth(diff(viconData(:,2)),40)./viconPer;
% gya = smooth(diff(gyv),40)./viconPer;
gyv = [0; gyv];
gya = [0; 0; gya];
gya([1:40,end-40:end])=0;

zv = meanFilter(diff(viconData(:,3)),10)./viconPer;
za = meanFilter(diff(zv),10)./viconPer;
% zv = smooth(diff(viconData(:,3)),20)./viconPer;
% za = smooth(diff(zv),20)./viconPer;
zv  = [0; zv];
za = [0; 0; za];
za([1:40,end-40:end])=0;

bv = convertGlobalToBody([gxv,gyv,zv,viconData(:,4)]);
bxv = bv(:,1);
byv = bv(:,2);


ba = convertGlobalToBody([gxa,gya,za,viconData(:,4)]);
bxa = ba(:,1);
bya = ba(:,2);

%%
% rawIMUData = d.IMU(2500:12500,:);
rawIMUData = d.IMU;

inds = findLatestsInds(rawIMUData(:,end),viconTimes);
% inds = viconData(:,3)>.3;
viconV = [bxv(inds,1),byv(inds,1),zv(inds,1)];
viconA = [bxa(inds,1),bya(inds,1),za(inds,1)];
viconrpy = viconrpy(inds,:);


%%%Done preprocessing the vicon data
dt = mean(diff(rawIMUData(:,1)/1e6));
imuRaw = rawIMUData(:,2:4);
imuquat = rawIMUData(:,8:11);
imurpy = bot_quat2rpy(imuquat);
imurpy(:,3)=0;
imuquat = bot_rpy2quat(imurpy);
viconrpy(:,3)=0;
viconquat = bot_rpy2quat(viconrpy);

imuSubG = zeros(size(imuRaw));
imuSubG2 = zeros(size(imuRaw));
for i = 1:size(imuRaw,1)
    imuSubG(i,:) = -(bot_quat_rotate(imuquat(i,:),imuRaw(i,:))-[0;0;-9.81]);
    imuSubG2(i,:) = -(bot_quat_rotate(viconquat(i,:),imuRaw(i,:))-[0;0;-9.81]);
end

bias = [smooth(viconA(:,1)-imuSubG(:,2),1000),...
        smooth(viconA(:,2)-imuSubG(:,3),1000),...
        smooth(viconA(:,3)-imuSubG(:,3),1000)];
plot([viconA(:,2),imuSubG(:,2)])