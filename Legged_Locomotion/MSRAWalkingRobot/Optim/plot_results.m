figure(1)
subplot(3,1,1)
plot(simout_torque.yout{2}.Values.Time,simout_torque.yout{2}.Values.Data,'b-');
grid on;
title('Robot Motion')
xlabel('Time [s]');
ylabel('Distance traveled [m]');
subplot(3,1,2)
plot(simout_torque.yout{1}.Values.Time,simout_torque.yout{1}.Values.Data,'b-');
grid on;
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
subplot(3,1,3)
plot(simout_torque.yout{3}.Values.Time,simout_torque.yout{3}.Values.Data,'b-');
grid on;
xlabel('Time [s]');
ylabel('Torso Height [cm]');

figure(2)
subplot(3,1,1)
plot(simout_torque.yout{4}.Values.ankle_torque.Time,simout_torque.yout{4}.Values.ankle_torque.Data,'r-');
grid on;
title('Joint Torques')
ylabel('Right Leg Ankle Torque [N*m]');
subplot(3,1,2)
plot(simout_torque.yout{4}.Values.knee_torque.Time,simout_torque.yout{4}.Values.knee_torque.Data,'r-');
grid on;
xlabel('Time [s]');
ylabel('Right Leg Knee Torque [N*m]');
subplot(3,1,3)
plot(simout_torque.yout{4}.Values.hip_torque.Time,simout_torque.yout{4}.Values.hip_torque.Data,'r-');
grid on;
xlabel('Time [s]');
ylabel('Right Leg Hip Torque [N*m]');

figure(3)
subplot(3,1,1)
plot(simout_torque.yout{4}.Values.ankle_angle.Time,simout_torque.yout{4}.Values.ankle_angle.Data,'k-');
grid on;
title('Joint Angles')
xlabel('Time [s]');
ylabel('Right Leg Ankle Angle [deg]');
subplot(3,1,2)
plot(simout_torque.yout{4}.Values.knee_angle.Time,simout_torque.yout{4}.Values.knee_angle.Data,'k-');
grid on;
xlabel('Time [s]');
ylabel('Right Leg Knee Angle [deg]');
subplot(3,1,3)
plot(simout_torque.yout{4}.Values.hip_angle.Time,simout_torque.yout{4}.Values.hip_angle.Data,'k-');
grid on;
xlabel('Time [s]');
ylabel('Right Leg Hip Angle [deg]');
