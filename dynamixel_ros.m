%% clc
clear all
clc
%% connect matlab and ros
rosinit
%% rosnode
rosnode list

%% rostopic
rostopic list 
%% rosservice
rosservice list
%% get information about /turtle1/pose
rostopic info /dxl_state_topic

%% see what data is published on the topic
vel = rostopic('echo','/dxl_state_topic')
showdetails(vel)

%% rosmsg show
rosmsg show my_dynamixel_workbench_test/dxl_state

%% create a subscriber for /turtle1/pose topic
scanner = rossubscriber('/dxl_state_topic')
%% dealwith data dxl_state topic
clc
clear
filename='data/04.txt';
[time,radian,velocity,current]=textread(filename,'%*s %f %*s %d %*s %f %*s %f','delimiter',':');
t=1:1:length(radian);
plot(t,radian')
%% dealwith data dynamixel_statelist topic
% uint: current:2.69[mA]  velocity:0.229[rev/min]  position:1[pulse] 0.088°/Value
clc
clear
filename='data/11.txt';
[time,radian1,velocity1,current1,radian2,velocity2,current2,radian0,velocity0,current0]...
=textread(filename,'%f %*s %*d %d %d %d %*s %*d %d %d %d %*s %*d %d %d %d','delimiter',',');
t=1:1:length(radian1);
figure(100),
plot(t,radian1');
hold on
plot(t,radian2');
plot(t,radian0');
figure(200),
plot(t,velocity1');
hold on
plot(t,velocity2');
plot(t,velocity0');
figure(300),
plot(t,current1');
hold on
plot(t,current2');
plot(t,current0');
%% output trajectory yaml file
clc
fid = fopen('motorTra.yaml','w');
len = length(motorTheta);
motorNum = 3;
wayPointNum = len;
thetaData=motorTheta(1:wayPointNum,1:motorNum);
rowIndex = 1;
colIndex = 1;
startTime = 0;
fprintf(fid,'joint:\n');
fprintf(fid,'  names: [zero,first,second]\n');
fprintf(fid,'trajectory:\n');
fprintf(fid,'  index: [');
for i=1:(wayPointNum-1)
    fprintf(fid,'wp%d,',i);
end
fprintf(fid,'wp%d]\n',wayPointNum);
for i=1:wayPointNum
    fprintf(fid,'  wp%d:\n',i);
    fprintf(fid,'    pos: [');
    for j=1:(motorNum-1)
       % rowIndex,colIndex
        fprintf(fid,'%f,',thetaData(rowIndex,colIndex));
        colIndex=colIndex+1;
    end
    %rowIndex,colIndex
    fprintf(fid,'%f]\n',thetaData(rowIndex,colIndex));
    rowIndex=rowIndex+1;
    colIndex = 1;
    startTime = startTime+0.2;
    fprintf(fid,'    time_from_start: %f\n',startTime);
end
fclose(fid);

