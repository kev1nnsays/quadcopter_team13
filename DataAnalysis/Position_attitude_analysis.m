%% Load and preprocess data
clear
IMU = csvread('imu.csv');
%    imudata.gyro_x = gyro_data[0];
%     imudata.gyro_y = gyro_data[1];
%     imudata.gyro_z = gyro_data[2];
%     imudata.accel_x = accel_data[0];
%     imudata.accel_y = accel_data[1];
%     imudata.accel_z = accel_data[2];
[m,n] = size(IMU);
IMU_start_t = IMU(1,1);
for i = 1:m
    IMU(i,1) = (IMU(i,1)-IMU_start_t)/1000000; %change to seconds
    IMU(i,2:4) = IMU(i,2:4).*(pi/180); 
end

MCAP = csvread('mcap.csv');
% #ifdef GEOFENCE_AXES
%     mc.pose[0] = (double)optmsg.z;
%     mc.pose[1] = -(double)optmsg.x;
%     mc.pose[2] = (double)optmsg.y;
%     mc.pose[3] = euler[2];
%     mc.pose[4] = -euler[0];
%     mc.pose[5] = euler[1];
%     
% #else
%     // Aircraft coordinate version
%     // Optitrack coordinates:  (x "forward" (matches aircraft +x), 
%     // y "up" (matches aircraft -z), z "right" (matches aircraft +y)
%     mc.pose[0] = (double)optmsg.x;
%     mc.pose[1] = (double)optmsg.z;
%     mc.pose[2] = -(double)optmsg.y;
%     mc.pose[3] = euler[0];
%     mc.pose[4] = euler[2];
%     mc.pose[5] = -euler[1];
[m,n] = size(MCAP);
MCAP_start_t = MCAP(1,1);
for i = 1:m
    MCAP(i,1) = (MCAP(i,1)-MCAP_start_t);
end


%% Data Analysis
% MCAP DATA
[m,n] = size(MCAP);
percentage = 0.05;
m_temp = floor(percentage*m);
subplot(2,2,1)
plot3(MCAP(1:m_temp,2),MCAP(1:m_temp,3),MCAP(1:m_temp,4),'.g')
xlabel('x')
ylabel('y')
zlabel('z')
title('3D Position from Optitrack')
hold on

subplot(2,2,2)
plot(MCAP(:,1),MCAP(:,2),'-r')
xlabel('Time (sec)')
ylabel('X Position')
title('MCAP: X Position Over Time')
hold on

subplot(2,2,3)
plot(MCAP(:,1),MCAP(:,3),'-b')
xlabel('Time (sec)')
ylabel('Y Position')
subplot(2,2,4)
plot(MCAP(:,1),MCAP(:,4),'-y')
xlabel('Time (sec)')
ylabel('Z Position')

figure()
plot(MCAP(:,1),MCAP(:,5),'-r')
hold on
plot(MCAP(:,1),MCAP(:,6),'-g')
hold on
plot(MCAP(:,1),MCAP(:,7),'-b')
xlabel('Time (sec)')
ylabel('Angle (radians)')
legend('Euler Angle about X axis','Euler Angle about y axis', 'Euler Angle about z axis')
title('MCAP Euler Angles')

[m,n] = size(IMU);
m_temp = floor(percentage*m);
% figure()
% plot3(IMU(1:m_temp,5),IMU(1:m_temp,6),IMU(1:m_temp,7),'.r')
% xlabel('xdt^2')
% ylabel('ydt^2')
% zlabel('zdt^2')


%% Comparison between IMU and MCAP
figure()
plot(IMU(:,1),IMU(:,2),'-r') %gyro about x, IMU
hold on
plot(MCAP(:,1),MCAP(:,5),'-b') %gyro about x, IMU
title('Pitch Comparison')
xlabel('Time (sec)')
ylabel('Pitch Angle (rad)')
legend('IMU Pitch','MCAP Pitch')

figure()
plot(IMU(:,1),IMU(:,3),'-r') %gyro about y, IMU
hold on
plot(MCAP(:,1),MCAP(:,6),'-b') %euler about y, MCAP
title('Roll Comparison')
xlabel('Time (sec)')
ylabel('Roll Angle (rad)')
legend('IMU Roll','MCAP Roll')
figure()
plot(IMU(:,1),IMU(:,4),'-r') %gyro about z, IMU
hold on
plot(MCAP(:,1),MCAP(:,7),'-b') %euler about z, MCAP
title('Yaw Comparison')
xlabel('Time (sec)')
ylabel('Yaw Angle (rad)')
legend('IMU Yaw','MCAP Yaw')

