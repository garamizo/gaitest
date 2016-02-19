%% Load data

clear; clc

len = @(v) sqrt(sum(v.^2, 2));
norm1 = @(v) bsxfun(@times, v, 1./len(v));

data1 = importfile1('~/catkin_ws2/imu20.csv', 2, Inf);
data2 = importfile('~/catkin_ws2/coef20.csv', 2, Inf);

%% Parse data
t1 = data1(:,1)*1e-9;
% quat1 = data1(:,[8 5:7]);
% quat1 = bsxfun(@times, quat1, sign(quat1(:,4)));
% quat1 = circshift(quat1, 0);
% 
% t2 = (1:a.len)/300 + 1.0560;
% quat2 = a.body(1).quat;
% 
% plot(t1-t1(1), quat1)
% reset(gca); hold on; plot(t2, quat2, '--')

quat = data1(:,[8 5:7]);

t2 = data2(:,1)*1e-9;
coef = data2(:,5:8);

t = t1 - t1(1);

%% Replace IMU data with mocap

tt1 = (data1(:,1)-data1(1,1))*1e-9;
qBs = data1(:,[8 5:7]);
qBs = bsxfun(@times, qBs, sign(qBs(:,4)));
qBs = circshift(qBs, 0);

tt2 = (1:a.len)/300 + 1.0560;
quat2 = a.body(1).quat;

qbc = interp1(tt2, quat2, tt1, 'pchip', NaN);
qbc = bsxfun(@times, qbc, sign(qbc(:,1)));

P1 = quatrotate(qBs, [0 0 1]); % up in imu moving frame (B)
P2 = quatrotate(qbc, [0 1 0]); % up in cam moving frame (b)
rows = (tt1>2 & tt1<8) & ~any(isnan([P1 P2]), 2);
[ret_R, ret_t] = rigid_transform_3D(P1(rows,:), P2(rows,:));
qBb = dcm2quat(ret_R.');

% figure; plot(t1, quatrotate(qBs, [0 0 1]))
% reset(gca); hold on; plot(t1, quatrotate(quatmultiply(qbc, qBb), [0 1 0]), '--')

% heading in imu inertial frame (s)
P1 = [  quatrotate(quatinv(qBs), [1 0 0])
        quatrotate(quatinv(qBs), [0 1 0])
        quatrotate(quatinv(qBs), [0 0 1]) ];
% heading in cam inertial frame (c)
P2 = [  quatrotate(quatinv(quatmultiply(qbc, qBb)), [1 0 0])
        quatrotate(quatinv(quatmultiply(qbc, qBb)), [0 1 0])
        quatrotate(quatinv(quatmultiply(qbc, qBb)), [0 0 1]) ];
rows = repmat((tt1>2 & tt1<8), [3 1]) & ~any(isnan([P1 P2]), 2);
[ret_R, ret_t] = rigid_transform_3D(P1(rows,:), P2(rows,:));
qcs = dcm2quat(ret_R);

% figure; plot(P1) % 
% reset(gca); hold on; plot(quatrotate(quatinv(qcs), P2), '--')

qBbcs = quatmultiply(qcs, quatmultiply(qbc, qBb));
quat = qBbcs;

% figure; plot(t1, qBs) % 
% reset(gca); hold on; plot(t1, qBbcs, '--')

%% Main calculation

% resample
nanrows = any(isnan(coef), 2);
coef = bsxfun(@times, coef(~nanrows,:), -sign(coef(~nanrows,4)));
coef = interp1(t2(~nanrows), coef(~nanrows,:), t1, 'pchip');

% Crop data
rows = t < 1.5 | t > 9;
t(rows) = [];
quat(rows,:) = [];
coef(rows,:) = [];

% acc = data1(:,30:32);
% 
% angVel = data1(:,18:20);
% accel = data1(:,30:32);
% 
% factor = 0.7;
% dir = repmat([0 0 1], [size(data1, 1) 1]);
% for k = 2 : size(accel,1)
%     
%     accelVec = accel(k,:) / 9.8;
%     dt = t1(k) - t1(k-1);
%     
%     dir(k,:) = factor*(dir(k-1,:) - cross(angVel(k,:), dir(k-1,:))*dt) + (1-factor)*accelVec;
%     dir(k,:) = dir(k,:) / sqrt(sum(dir(k,:).^2));
% end

nvec = norm1(coef(:,1:3));

% 
% Fs = 1./mean(diff(t));
% [B,A] = butter(3, 20/Fs);
% nvec = norm1(filtfilt(B, A, nvec));

% disregard yaw rotation
[r1, r2, r3] = quat2angle(quat, 'ZYX');
quatn = angle2quat(r1*0, r2, r3, 'ZYX');
% q0 = norm1(mean(quat(1:50,:)));
% quatn = quatmultiply(quat, quatinv(q0));
quatn(:,3) = -quatn(:,3); % fix???
qcs = norm1([0 -1 0 0]);
qcw = quatmultiply(qcs, quatn);

% ground plane
ref = [0 0 -1]; % norm1(mean(quatrotate(quatinv(qcw), nvec)));
crossProd = bsxfun(@cross, ref.', quatrotate(quatinv(qcw), nvec).').';
sth = len(crossProd);
cth = sum(repmat(ref, [length(t) 1]) .* quatrotate(quatinv(qcw), nvec), 2);
th = atan2(sth, cth);
gndAngle = bsxfun(@times, norm1(crossProd), th);

% Segment =================================================================
[~,idx] = findpeaks(-r2, 'MinPeakProminence', 0.3);
glen = diff(idx);
idx(end) = [];

x = (1:mean(glen))/mean(glen);
roll = zeros(length(x), length(glen));
pitch = zeros(length(x), length(glen));

for k = 1 : length(glen)
    rows = idx(k)+(1:glen(k));

    y = gndAngle(rows,:)*180/pi;
    roll(:,k) = interp1((1:glen(k))/glen(k), y(:,1), x, 'pchip');
    pitch(:,k) = interp1((1:glen(k))/glen(k), y(:,2), x, 'pchip');
end

%% Present

figure
plot(t, nvec)
reset(gca); hold on;  plot(t, quatrotate(qcw, [0 0 -1]), '--')
ylabel('Acquired/Expected normal vectors')
grid on
% legend('Acquired X', 'Acquired Y', 'Acquired Z', 'Expected X', 'Expected Y', 'Expected Z')

figure
subplot(211); hold on
plot(x, roll)
shadedErrorBar(x, mean(roll,2), std(roll,[],2),'-b', 1);
ylim([-10 10]); ylabel('Roll [deg]'); grid on
set(gca, 'XTick',[0.1 0.4 0.8], ...
    'XTickLabel',{'heel-strike' 'mid-stance' 'mid-swing'}, ...
    'XTickLabelRotation', -25)
subplot(212); hold on
plot(x, pitch)
shadedErrorBar(x, mean(pitch,2), std(pitch,[],2),'-r', 1);
ylim([-10 10]); ylabel('Pitch [deg]'); grid on
set(gca, 'XTick',[0.1 0.4 0.8], ...
    'XTickLabel',{'heel-strike' 'mid-stance' 'mid-swing'}, ...
    'XTickLabelRotation', -25)


figure
for k = 1 : length(glen)
    rows = idx(k)+(1:glen(k));
    
    subplot(211); plot(t(rows), gndAngle(rows,1)*180/pi); hold on
    subplot(212); plot(t(rows), gndAngle(rows,2)*180/pi); hold on
end
