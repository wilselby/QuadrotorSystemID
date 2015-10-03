%% Setup Path
addpath('~/carmen3D/src/matlab:', ...
    '~/carmen3D/bin:', ...
    '~/carmen3D/src/controller/sysidAnalysis/rls/matlab/:');


%% Extract Data
% expects log data structure named 'd'
% stateField = 'STATE_ESTIMATE';
% stateField = 'STATE_GROUND_TRUTH'
stateField = 'POSE';

% % handle out of vicon errors:
% nani = find(isnan(d.(stateField)(:,4)),1);
% if isempty(nani)
%     nani = size(d.(stateField),1);
% end
% vinds= 1:(nani-1);
% vinds = 4000:10000;
vinds = 1:size(d.(stateField),1);

% extract vicon data
% viconTimes = d.(stateField)(vinds,end-1);
% viconData = d.(stateField)(vinds,:);

if (strmatch(stateField,'POSE'))
    %hack to make it look like old state estimator
    viconTimes = d.(stateField)(vinds,1)/1e6;
    viconData = d.(stateField)(vinds,[2:4,14,13,12,5:7,15:17,15:17]);
end

if (strmatch(stateField,'STATEPOS'))
    viconData(:,1:3) = viconData(:,1:3) /1000;
end

% remove inds where we're on the ground...
hinds = viconData(:,3)>.3;
viconTimes=viconTimes(hinds,:);
viconData=viconData(hinds,:);

% extract control data
esinds = d.QUAD_RC_DATA(:,1)/1e6>= viconTimes(1) & d.QUAD_RC_DATA(:,1)/1e6<=viconTimes(end);
eservoTimes = d.QUAD_RC_DATA(esinds,1)/1e6;
eservoData = d.QUAD_RC_DATA(esinds,[3 4 5 6]);
eservoData(:,[1,2,4])=eservoData(:,[1,2,4])/2048 -1;
eservoData(:,3) = eservoData(:,3)/4096;
% 
% batInds = findLatestsInds(eservoTimes,d.SYSTEM_VITALS(:,1)/1e6);
% batteryTimes = d.SYSTEM_VITALS(batInds,1)/1e6;
% battery = d.SYSTEM_VITALS(batInds,2);

if (isfield(d,'SYSTEM_VITALS'))
    batInds = findLatestsInds(eservoTimes,d.SYSTEM_VITALS(:,1)/1e6);
    batteryTimes = d.SYSTEM_VITALS(batInds,1)/1e6;
    battery = d.SYSTEM_VITALS(batInds,2);
else %handle older logs where the system vitals format changed...
    batteryTimes = eservoTimes;
    battery = 11.5*ones(size(eservoTimes));
end

eservoTimes = eservoTimes-viconTimes(1);
% batteryTimes = batteryTimes - viconTimes(1);
viconTimes = viconTimes-viconTimes(1);

bodyStates = convertGlobalToBody(viconData);

viconPer = median(diff(viconTimes));


%% Differentiate & Filter

% tbd - decide between meanFilter & smooth
% gxv = meanFilter(diff(viconData(:,1)),20)./viconPer;
% gxa = meanFilter(diff(gxv),20)./viconPer;
gxv = smooth(diff(viconData(:,1)),40)./viconPer;
gxa = smooth(diff(gxv),40)./viconPer;
gxv = [0; gxv];
gxa = [0; 0; gxa];
gxa([1:40,end-40:end])=0; %get rid of end effects...

% gyv = meanFilter(diff(viconData(:,2)),20)./viconPer;
% gya = meanFilter(diff(gyv),20)./viconPer;
gyv = smooth(diff(viconData(:,2)),40)./viconPer;
gya = smooth(diff(gyv),40)./viconPer;
gyv = [0; gyv];
gya = [0; 0; gya];
gya([1:40,end-40:end])=0;

% zv = meanFilter(diff(viconData(:,3)),10)./viconPer;
% za = meanFilter(diff(zv),10)./viconPer;
zv = smooth(diff(viconData(:,3)),20)./viconPer;
za = smooth(diff(zv),20)./viconPer;
zv  = [0; zv];
za = [0; 0; za];
za([1:40,end-40:end])=0;

bv = convertGlobalToBody([gxv,gyv,zv,viconData(:,4)]);
bxv = bv(:,1);
byv = bv(:,2);


ba = convertGlobalToBody([gxa,gya,za,viconData(:,4)]);
bxa = ba(:,1);
bya = ba(:,2);

pp = medianFilter(viconData(:,5),10);  
rp = medianFilter(viconData(:,6),10);


tv = medianFilter(diff(viconData(:,4)),10)./viconPer;
tv = [0; tv];

% plot([bxv,bxa])


%% Find Delays
% search for best alignment of vicon and control data for each control
% channel
offsets = linspace(-1,.1,50);

RMSp = zeros(size(offsets));
RMSr = zeros(size(offsets));
RMSz = zeros(size(offsets));
RMSt = zeros(size(offsets));

i = 0;
for o = offsets
    i = i+1;
    vinds = findLatestsInds(eservoTimes,viconTimes+o);

    %     RMSp(i) = norm(pp(vinds)*5+eservoData(:,1));
    ccf = corrcoef([pp(vinds),eservoData(:,1)]);
    RMSp(i) = ccf(1,2);

    %     RMSr(i) = norm(rp(vinds)*5-eservoData(:,2));
    ccf = corrcoef([-rp(vinds),eservoData(:,2)]);
    RMSr(i) = ccf(1,2);

    %     RMSz(i) = norm(za(vinds)*10-eservoData(:,3));
    ccf = corrcoef([za(vinds),eservoData(:,3)]);
    RMSz(i) = ccf(1,2);

    %     RMSt(i) = norm(tv(vinds)-eservoData(:,4));
    ccf = corrcoef([tv(vinds),eservoData(:,4)]);
    RMSt(i) = ccf(1,2);

end

[v oi] = max(RMSp);
vindsp = findLatestsInds(eservoTimes,viconTimes+offsets(oi));
[v oi] = max(RMSr);
vindsr = findLatestsInds(eservoTimes,viconTimes+offsets(oi));
[v oi] = max(RMSz);
vindsz = findLatestsInds(eservoTimes,viconTimes+offsets(oi));
[v oi] = max(RMSt);
vindst = findLatestsInds(eservoTimes,viconTimes+offsets(oi));


% tbd - perhaps get finddelay to work

% -- unused
%find the delay using the matlab builtin finddelay
vinds = findLatestsInds(eservoTimes,viconTimes);
pdelay = finddelay(pp(vinds),eservoData(:,1));
rdelay = finddelay(-rp(vinds),eservoData(:,2));
zdelay = finddelay(za(vinds),eservoData(:,3));
tdelay = finddelay(tv(vinds),eservoData(:,4));
% --

% %use a delay of 0
% vinds = findLatestsInds(eservoTimes,viconTimes);
% pdelay = 0;
% rdelay = 0;
% zdelay = 0;
% tdelay = 0;
% 
% vindsp = findLatestsInds(eservoTimes,viconTimes+pdelay*viconPer);
% vindsr = findLatestsInds(eservoTimes,viconTimes+rdelay*viconPer);
% vindsz = findLatestsInds(eservoTimes,viconTimes+zdelay*viconPer);
% vindst = findLatestsInds(eservoTimes,viconTimes+tdelay*viconPer);

esg = convertBodyToGlobal([eservoData(:,1:3),viconData(vindst,4)]);

% plot vicon trajectory
figure(33)
plot3(viconData(:,1),viconData(:,2),viconData(:,3),'ro');axis equal

% plot delays
figure(34)
plot(offsets,[RMSp',RMSr',RMSz',RMSt']);
xlabel('time offset between command and action')
legend('pitch','roll','z','Theta')

figure(35)
subplot(4,1,1);
plot([pp(vindsp),eservoData(:,1)])
title('aligned pitch vs. eservo')

subplot(4,1,2);
plot([rp(vindsr),eservoData(:,2)])
title('aligned roll vs. eservo')

subplot(4,1,3);
plot([za(vindsz),eservoData(:,3)])
title('aligned z acc vs. eservo')


subplot(4,1,4);
plot([tv(vindst),eservoData(:,4)])
title('aligned yaw vel vs. eservo')

%% Least Squares Fit
figure(350)

% x_accel = m * pitch + b    (kx = [m, b])
Ax=[eservoData(:,1),ones(size(eservoData,1),1)];
Ax2=[eservoData(:,1),bxv(vindsp),ones(size(eservoData,1),1)];
Bx = bxa(vindsp);
% kx = Ax\Bx;
% kx2 = Ax2\Bx;
[kx,loosx,bestlambdasx] = lrlsloobest(Ax, Bx);
[kx2,loosx2,bestlambdasx2] = lrlsloobest(Ax2, Bx);
subplot(4,1,1);
plot(Ax(:,1), Bx(:), '.'); hold on;
x_x = [min(Ax(:,1))-1:.01:max(Ax(:,1))+1];
y_x = kx(1)*x_x + kx(2);
plot(x_x, y_x);

% y_accel = m * roll + b     (ky = [m, b])
Ay=[eservoData(:,2),ones(size(eservoData,1),1)];
Ay2=[eservoData(:,1:4),byv(vindsr),bxv(vindsp),zv(vindsz),tv(vindst),ones(size(eservoData,1),1)];
Ay2 = [Ay2.^2,Ay2];
By = bya(vindsr);
% ky = Ay\By;
% ky2 = Ay2\By;
[ky,loosy,bestlambdasy] = lrlsloobest(Ay, By);
[ky2,loosy2,bestlambdasy2] = lrlsloobest(Ay2, By);
subplot(4,1,2);
plot(Ay(:,1), By(:), '.'); hold on;
x_y = [min(Ay(:,1))-1:.01:max(Ay(:,1))+1];
y_y = ky(1)*x_y + ky(2);
plot(x_y, y_y);

Az=[eservoData(:,3),ones(size(eservoData,1),1)];
% z_accel = m1 * thrust + m2 * battery_voltage + b     (kz2 = [m1,m2,b])
% Az2=[eservoData(:,3),battery,ones(size(eservoData,1),1)];
% Az3=[eservoData(:,3),zv(vindsz),battery,ones(size(eservoData,1),1)];
Bz = za(vindsz);
% kz = Az\Bz;
% kz2 = Az2\Bz;
[kz,loosz,bestlambdasz] = lrlsloobest(Az, Bz);
% [kz2,loosz2,bestlambdasz2] = lrlsloobest(Az2, Bz);
% [kz3,loosz3,bestlambdasz3] = lrlsloobest(Az3, Bz);
subplot(4,1,3);
plot(Az2(:,1), Bz(:), '.'); hold on;
x_z = [min(Az2(:,1))-1:.01:max(Az2(:,1))+1];
y_z = kz(1)*x_z + kz(2);
plot(x_z, y_z);

At=[eservoData(:,4),ones(size(eservoData,1),1)];
% yaw_vel = m * yaw + b    (kt = [m, b])
At2=[eservoData(:,4),ones(size(eservoData,1),1)];
Bt = tv(vindst);
% kt = At\Bt;
% kt2 = At2\Bt;
[kt,loost,bestlambdast] = lrlsloobest(At, Bt);
[kt2,loost2,bestlambydast2] = lrlsloobest(At2, Bt);
subplot(4,1,4);
plot(At(:,1), Bt(:), '.'); hold on;
x_t = [min(At(:,1))-1:.01:max(At(:,1))+1];
y_t = kt(1)*x_t + kt(2);
plot(x_t, y_t);

% used for kalman process model
%find conversion from control input to tilt angle

Ap=[eservoData(:,1),ones(size(eservoData,1),1)];
Bp = pp(vindsp);
kp = Ap\Bp;

Ar=[eservoData(:,2),ones(size(eservoData,1),1)];
Br = rp(vindsr);
kr = Ar\Br;

%find conversion from pitch angle to aceleration

Apx=[pp,ones(size(pp,1),1)];
Bpx = bxa;
kpx = Apx\Bpx;

Ary=[rp,ones(size(rp,1),1)];
Bry = bya;
kry = Ary\Bry;


%try fitting to global accelerations.. doesn't seem to work
Axg=[esg(:,1),ones(size(eservoData,1),1)];
Ax2g=[esg(:,1),gxv(vindsp),ones(size(eservoData,1),1)];
Bxg = gxa(vindsp);
[kxg,loosxg,bestlambdasxg] = lrlsloobest(Axg, Bxg);
[kx2g,loosx2g,bestlambdasx2g] = lrlsloobest(Ax2g, Bxg);




disp(['kx= [' num2str(kx') '];']);
disp(['ky= [' num2str(ky') '];']);
disp(['kz= [' num2str(kz') '];']);
disp(['kt= [' num2str(kt') '];']);
disp(' ');
disp(['kp= [' num2str(kp') '];']);
disp(['kr= [' num2str(kr') '];']);
disp(['kpx= [' num2str(kpx') '];']);
disp(['kry= [' num2str(kry') '];']);

% controlBias = mean(eservoData(:,1:4));
% controlBias = -[kx(2)/kx(1),ky(2)/ky(1),kz(2)/kz(1),kt(2)/kt(1)];
controlBias = -[kx(2)/kx(1),ky(2)/ky(1),kz2(3)/kz2(1),kt(2)/kt(1)];
disp(['controlBias= [' num2str(controlBias) '];']);

batteryGain = -kz2(2)/kz2(1);
disp(['battery gain = ' num2str(batteryGain)]);

figure(36)
subplot(4,1,1);
plot(eservoTimes,[bxa(vindsp),Ax*kx,Ax2*kx2])
title('aligned x acc vs. eservo*Kx')

subplot(4,1,2);
plot(eservoTimes,[bya(vindsr),Ay*ky,Ay2*ky2])
title('aligned y acc vs. eservo*Ky')

subplot(4,1,3);
plot(eservoTimes,[za(vindsz),Az*kz,Az2*kz2])
title('aligned z acc vs. eservo*Kz')

subplot(4,1,4);
plot(eservoTimes,[tv(vindst),At*kt,At2*kt2])
title('aligned yaw vel vs. eservo*Kt')



figure(37)
subplot(4,1,1);
plot(eservoTimes,[pp(vindsp),eservoData(:,1)*kp(1)+kp(2)])
title('aligned pitch vs. eservo*Kp')

subplot(4,1,2);
plot(eservoTimes,[rp(vindsp),eservoData(:,2)*kr(1)+kr(2)])
title('aligned roll vs. eservo*Kr')

subplot(4,1,3);
plot(viconTimes,[bxa,pp*kpx(1)+kpx(2)]);
title('vicon acceleration vs pitch*kpx');

subplot(4,1,4);
plot(viconTimes,[bya,rp*kry(1)+kry(2)]);
title('vicon acceleration vs pitch*kry');

% computeLQRGains
computeLQRGains2
% printGainsForC
