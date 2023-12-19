bag=rosbag('robot_pose.bag');
bSel= select(bag,'Topic','/fra2mo/pose');
msgSTRUCT = readMessages(bag,'DataFormat','struct');
for i=1:1464
    x(i)=msgSTRUCT{i,1}.Pose.Position.X;
    y(i)=msgSTRUCT{i,1}.Pose.Position.Y;
    z(i)=msgSTRUCT{i,1}.Pose.Position.Z;
    eps_x(i)=msgSTRUCT{i,1}.Pose.Orientation.X;
    eps_y(i)=msgSTRUCT{i,1}.Pose.Orientation.Y;
    eps_z(i)=msgSTRUCT{i,1}.Pose.Orientation.Z;
    eta(i)=msgSTRUCT{i,1}.Pose.Orientation.W;
end

t=0:0.2:300;
figure()
subplot(2,1,1)
plot(t(1:length(x)),x)
hold on
plot(t(1:length(x)),y)
hold on
plot(t(1:length(x)),z)
hold on
legend('x','y','z')
grid
xlabel('Time [s]')
ylabel('Position [m]')
subplot(2,1,2)
plot(t(1:length(x)),eps_x)
hold on
plot(t(1:length(x)),eps_y)
hold on
plot(t(1:length(x)),eps_z)
hold on
plot(t(1:length(x)),eta)
hold on
legend('\epsilon_x','\epsilon_y','\epsilon_z','\eta')
grid
xlabel('Time [s]')
ylabel('Quaternion')
axis([0 300 -1.1 1.1])