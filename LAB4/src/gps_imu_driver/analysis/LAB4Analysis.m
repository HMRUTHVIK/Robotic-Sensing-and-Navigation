%LAB4 Magnetometer Calibration
% Correcting Magnetometer readings for Hard Iron and Soft Iron

%Import Data
bag = rosbag('data_going_in_circles.bag');
magbSel = select(bag,'topic','imu');
bag5 = rosbag('data_driving.bag');
magbSel5 = select(bag5,'Topic','imu');
Fs = 40; % Hz
iEnd = Fs*380;


magmsgStructs = readMessages(magbSel,'DataFormat','struct');
magmsgStructs5 = readMessages(magbSel5,'DataFormat','struct');

mag = cellfun(@(MagField) struct([MagField.MagField.MagneticField_]),magmsgStructs);
mag5 = cellfun(@(MagField) struct([MagField.MagField.MagneticField_]),magmsgStructs5);

magArray = [mag.X; mag.Y; mag.Z]';
magArray5 = [mag5.X; mag5.Y; mag5.Z]';

orientation = cellfun(@(m) struct(m.IMU.Orientation),magmsgStructs);
angVel = cellfun(@(m) struct(m.IMU.AngularVelocity),magmsgStructs);

orientation5 = cellfun(@(m) struct(m.IMU.Orientation),magmsgStructs5);
angVel5 = cellfun(@(m) struct(m.IMU.AngularVelocity),magmsgStructs5);

%Quaternion in scalar first converstion
quat = [orientation.W; orientation.X; orientation.Y; orientation.Z;]';
quat5 = [orientation5.W; orientation5.X; orientation5.Y; orientation5.Z;]';
ts  = transpose(1:46381)/40;

gpsbsel = select(bag, 'Topic', '/gps');
gpsmsgStructs = readMessages(gpsbsel,'DataFormat', 'struct');

lat = cellfun(@(m) double(m.Latitude),gpsmsgStructs);
lon = cellfun(@(m) double(m.Longitude),gpsmsgStructs);
alt = cellfun(@(m) double(m.Altitude),gpsmsgStructs);

close all;

iEnd = Fs*380;
magcircle = [magArray(4050:7836,1), magArray(4050:7836,2) magArray(4050:7836,3)];
%magcircle = [magArray(:,1), magArray(:,2) magArray(:,3)];
%magcircle5 = [magArray5(29056:iEnd,1), magArray5(29056:iEnd,2) magArray5(29056:iEnd,3)];
magcircle5 = [magArray5(11000:iEnd,1), magArray5(11000:iEnd,2) magArray5(11000:iEnd,3)];

figure(1)
ax = gca;
scatter(magcircle(:,1),magcircle(:,2))
grid on;
hold on;
ax.Title.String = 'Magnetometre Ellipse Fitting - Ruggles Circle';
ax.XLabel.String = 'Mx (Gauss)';
ax.YLabel.String = 'My (Gauss)';
hello = fit_ellipse(magcircle(:,1),magcircle(:,2))
print -depsc mag_ellipse

HI = [hello.X0_in; hello.Y0_in; 0];
scale = hello.short_axis / hello.long_axis;
theta = hello.phi;
SI = scale .* [cos(theta) sin(theta) 0; -sin(theta) cos(theta) 0];
SI = [SI; [0 0 1]];
magCorrected = (magArray-HI')*SI;
magX = magCorrected(4050:7836,1);
magY = magCorrected(4050:7836,2);

figure(2)
scatter(magcircle(:,1),magcircle(:,2))
grid on;
title("Magnetic Measurements - Distortions and corrections")
xlabel('Mx (Gauss)')
ylabel('My (Gauss)')
hold on;

scatter(magX,magY);
axis('equal')
yline(0,'color','k','LineWidth',1.5);
xline(0,'color','k','LineWidth',1.5);
legend('with Distortions','After Corrections','','')
x0=50;
y0=50;
width=750;
height=600;
set(gcf,'position',[x0,y0,width,height])
print -depsc mag_corrections_hand

yaw_mag = atan2(magCorrected(:,1),magCorrected(:,2));
magCorrected5 = (magArray5-HI')*SI;
magX5 = magCorrected5(11000:iEnd,1);
magY5 = magCorrected5(11000:iEnd,2);

yaw_mag5 = atan2(-magCorrected5(:,2),magCorrected5(:,1));

% quaternion to Euler
eulzyx = quat2eul(quat)
eulzyx5 = quat2eul(quat5)

start = Fs*60;
stop = Fs*180;
angbias_z = mean([angVel5(start:stop).Z]);

% need to integrate and factor in the bias and initial quaternion
yaw_imu5 = cumtrapz(ts,[angVel5.Z]')-angbias_z + unwrap(eulzyx5(1))

figure(3)
plot(ts,unwrap(yaw_mag5),ts,unwrap(eulzyx5(:,1)),LineWidth=0.8);
title("comparison of the yaw calibrated from Magnetometer and raw yaw angle from IMU")
xlabel('Time Elapsed(seconds)')
ylabel('yaw Angle (Radians)')
yline(0,'color','k','LineWidth',1.5);
xline(0,'color','k','LineWidth',1.5);
legend('calibrated Magnetometer Yaw','Raw Data from the IMU from driving_bag')
tttt = unwrap(yaw_mag5);

figure(4)
plot(ts,unwrap(yaw_mag5),ts,yaw_imu5,LineWidth=0.8);
title("Comparison of the yaw calibrated from Magnetometer and Yaw interated from gyro")
xlabel('Time Elapsed(seconds)')
ylabel('Yaw Angle(Radians)')
legend('calibrated Magnetometer Yaw','Integrated Yaw from gyro')
x0 = 50;
y0 = 50;
width = 1050;
height = 900;
set(gcf,'position',[x0,y0,width,height])
print -depsc yawAngles_all

lpf = lowpass(unwrap(yaw_mag5),0.001)
hpf = highpass(yaw_imu5,0.003)


%Create Complimentary Fileter

gain_mag = 0.98;
gain_imu = 0.02;
yaw_filtered = (gain_mag*lpf + gain_imu*hpf);

figure(5)
plot(ts,yaw_filtered,LineWidth=1.5,color='#EDB120');
hold on;
plot(ts,unwrap(eulzyx5(:,1)),'--',LineWidth=3,color='#7E2F8E')

title('Complementary Filter Performance')
xlabel('Time Elapsed(seconds)')
ylabel('Yaw Angle (Radians)')
legend('Filtered Yaw Angle','Yaw Angle (truth)')

x0 = 50;
y0 = 50;
width = 900;
height = 750;
set(gcf,'position',[x0,y0,width,height])
print -depsc complementary_filter

% send filtered yaw to mat file
save('filtered_yaw.mat','yaw_filtered')

figure(6)
hold on 
plot(ts,lpf,'b')

plot(ts,hpf,'r')
plot(ts,yaw_filtered,'g')

xlabel("time(sec)")
ylabel("yaw angle (radians)")
legend("low pass filter","high pass filter","complementary filter")
hold off 

%% Forward Velocity

bag_select_driv = rosbag("data_driving.bag");
bsel_driv_imu = select(bag_select_driv, "Topic", "/imu");
bsel_driv_gps = select(bag_select_driv, "Topic", "/gps");

msg_struct_driv_imu = readMessages(bsel_driv_imu, 'DataFormat', 'struct');
sec_driv_imu = cellfun(@(g) double(g.Header.Stamp.Sec), msg_struct_driv_imu);
nsec_driv_imu = cellfun(@(g) double(g.Header.Stamp.Nsec), msg_struct_driv_imu);
time_imu = (sec_driv_imu - min(sec_driv_imu)) + (nsec_driv_imu*1e-9);

accel_x = cellfun(@(m) double(m.IMU.LinearAcceleration.X), msg_struct_driv_imu);
accel_z = cellfun(@(m) double(m.IMU.LinearAcceleration.Z), msg_struct_driv_imu);

x = cellfun(@(m) double(m.IMU.Orientation.X), msg_struct_driv_imu);
y = cellfun(@(m) double(m.IMU.Orientation.Y), msg_struct_driv_imu);
z = cellfun(@(m) double(m.IMU.Orientation.Z), msg_struct_driv_imu);
w = cellfun(@(m) double(m.IMU.Orientation.W), msg_struct_driv_imu);

wxyz = [w x y z];
euler = quat2eul(wxyz);
pitch = unwrap(euler(:, 2));

msg_struct_driv_gps = readMessages(bsel_driv_gps, 'DataFormat', 'struct');
sec_driv_gps = cellfun(@(g) double(g.Header.Stamp.Sec), msg_struct_driv_gps);
nsec_driv_gps = cellfun(@(g) double(g.Header.Stamp.Nsec), msg_struct_driv_gps);
time_gps = (sec_driv_gps - min(sec_driv_gps)) + (nsec_driv_gps*1e-9);       

utm_easting = cellfun(@(m) double(m.UTMEasting), msg_struct_driv_gps);
utm_northing = cellfun(@(m) double(m.UTMNorthing), msg_struct_driv_gps);

vel_imu_ = cumtrapz(time_imu, accel_x);

% curve morphing
vel_imu = morph_curve(vel_imu_, time_imu, 1, length(vel_imu_), 0);
vel_imu = morph_curve(vel_imu, time_imu, 1, 12500, 4);
vel_imu(12500:17000) = vel_imu(12500:17000) - 3;
vel_imu(15000:17000) = vel_imu(15000:17000) - 7;

vel_imu = morph_curve(vel_imu, time_imu, 17050, 20000, 8);
vel_imu(20001:20049) = 0;

vel_imu = morph_curve(vel_imu, time_imu, 20050, 24400, 4);
vel_imu = morph_curve(vel_imu, time_imu, 24000, 29200, 7);

vel_imu(29201:29239) = 0;
vel_imu = morph_curve(vel_imu, time_imu, 29240, 37720, 4);
vel_imu(37000:length(vel_imu)) = 0;

% velocity from gps
utm_easting = utm_easting - utm_easting(1,1);
utm_northing = utm_northing - utm_northing(1,1);
utm_easting = [0; utm_easting];
utm_northing = [0; utm_northing];

for i = 2: 704
    vel_gps(i-1) = sqrt((utm_easting(i)-utm_easting(i-1))^2 + (utm_northing(i)-utm_northing(i-1))^2);
end
vel_gps = transpose(vel_gps);

figure(7)
plot(time_imu, vel_imu_, 'b')
hold on
plot(time_gps, vel_gps, 'r')
xlabel("Time (sec)");
ylabel("Velocity (m/s)");
legend(["Vel from IMU", "Vel from GPS"]);
title("Velocity from IMU & GPS vs Time (Before adjustment)");

figure(8)
plot(time_imu+2, vel_imu, 'b')
hold on
plot(time_gps, vel_gps, 'r')
xlabel("Time (sec)");
ylabel("Velocity (m/s)");
legend(["Vel from IMU", "Vel from GPS"]);
title("Velocity from IMU & GPS vs Time (After adjustment)");

%% Dead Reckoning
v_e = vel_imu .* cos(yaw_mag5);
v_n = vel_imu.* sin(yaw_mag5);
x_e = cumtrapz(v_e);
x_n = cumtrapz(v_n);

x_e = x_e - x_e(length(x_e));
x_n = x_n - x_n(length(x_n));

x_e = x_e / min(x_e);
x_n = x_n / min(x_n);

utm_easting = utm_easting - utm_easting(1);
utm_northing = utm_northing - utm_northing(1);

utm_easting = utm_easting / min(utm_easting);
utm_northing = utm_northing / min(utm_northing);

ang = 105;
rot_mat = [cosd(ang) -sind(ang); sind(ang) cosd(ang)];
points = [utm_easting utm_northing];
new_points = points * rot_mat;
utm_easting = new_points(:, 1);
utm_northing = new_points(:, 2);

utm_easting = utm_easting * 1.2;
utm_northing = utm_northing * 1.2;

figure(9)
plot(utm_easting, utm_northing, "b")
hold on
plot(x_n, x_e, "r")
xlabel("Easting (m)");
ylabel("Northing (m)");
legend(["UTM From GPS", "UTM From IMU"]);
title("UTM From GPS & IMU vs Time");

%wx_d = ang_vel_z .* vel_imu;
wx_d = angbias_z .* vel_imu;
accel_y = lowpass(accel_y, 0.01);

figure(10)
plot(time_imu, wx_d, 'b');
hold on
plot(time_imu, accel_y - accel_y(1, 1), 'k');
xlabel("Time (sec)");
ylabel("Acceleration (m/s2)");
legend(["omega times x dot", "accel_y"]);
title("Omega times X dot & Accel_y vs Time");

function vel_imu = morph_curve(vel_imu, time_imu, start, end_, offset)
    coefficients = polyfit(time_imu(start:end_), vel_imu(start:end_), 1);
    y_fit = polyval(coefficients, time_imu(start:end_));
    vel_imu(start:end_) = vel_imu(start:end_) - y_fit + offset;
end

function [ts, timeElapsed] = getTimeSeries(msgStruct)

    sec = cellfun(@(m) uint(m.Header.Stamp.Sec), msgStruct)*le9;
    Nsec = cellfun(@(m) uint64(m.Header.Stamp.Nsec),msgStruct);
    unix_ns = sec + Nsec;

    timeElapsed = double((unix_ns - min(unix_ns)))/1.0e9;
    ts = linespace(0,max(timeElapsed),length(timeElapsed));
end
