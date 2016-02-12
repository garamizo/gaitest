[data,vname] = tblread('imu.csv', ',');
t = data(:,2)*1e-9;
quat = data(:,[7 4:6]);

[data,vname] = tblread('coef.csv', ',');
t2 = data(:,2)*1e-9;
coef = interp1(t2, data(:,4:7), t, 'pchip');
nvec = bsxfun(@times, coef(:,1:3), 1./sqrt(sum(coef(:,1:3).^2,2)));
% Fs = 1./mean(diff(t));
% [B,A] = butter(15, 5/Fs);
% nvec = filtfilt(B, A, nvec);
nvec = filtfilt(ones(15,1)/15, 1, nvec);

t = t - t(1);

% disregard yaw rotation
[r1, r2, r3] = quat2angle(quat, 'ZXY');
quatn = angle2quat(r1*0, r2, r3, 'ZXY');
quatn(:,2) = -quatn(:,2); % fix???
qcs = [0 0 1 0];
qcw = quatmultiply(qcs, quatn);

close; figure
plot(t, quatrotate(quatinv(qcw), nvec))

vec = bsxfun(@cross, [0 0 -1].', quatrotate(quatinv(qcw), nvec).').';
cth = sum(repmat([0 0 -1], [length(t) 1]) .* quatrotate(quatinv(qcw), nvec), 2);
qerr = [cth vec];

theta = asin(sin(2*acos( qerr(:,1) )));
vec = qerr(:,2:4) ./ repmat(sqrt(1-qerr(:,1).^2), [1 3]);
angleErr = vec .* repmat(theta, [1 3]);

theta = asin(sin(2*acos( quatn(:,1) )));
vec = quatn(:,2:4) ./ repmat(sqrt(1-quatn(:,1).^2), [1 3]);
angle = vec .* repmat(theta, [1 3]);

nwin = 10;
stdErr = [ZTools2.movingstd(angleErr(:,1),nwin,'central') ...
    ZTools2.movingstd(angleErr(:,2),nwin,'central') ...
    ZTools2.movingstd(angleErr(:,3),nwin,'central')];

subplot(211); shadedErrorBar(t, angle(:,1)*180/pi, stdErr(:,1)*180/pi,'-r', 1);
ylabel('Roll [deg]')
grid on; xlim([t(1)+2 t(end)-2]); ylim([-20 40])
subplot(212); shadedErrorBar(t, angle(:,2)*180/pi, stdErr(:,2)*180/pi,'-b', 1); 
ylabel('Pitch [deg]'); xlabel('time [s]')
grid on; xlim([t(1)+2 t(end)-2]); ylim([-20 40])
