%% 图论最短路径
clear all
A=[0 1 0 2 0 0 ;
      0 0 3 4 0 0 ;
      0 -2 0 5 1 0;
      0 4 0 0 -3 0;
      0 0 2 3 0 0;
      0 0 2 0 2 0];
  G=digraph(A,{'V1','V2','V3','V4','V5','V6'});
  TR=shortestpathtree(G,1);
  p = plot(G,'EdgeLabel',G.Edges.Weight);
  highlight(p,TR,'EdgeColor','r');
%%   input学习
clear all
C = input('Enter a num\n');
F = (C*1.8)+32;
fprintf('华氏温度=%.2f\n',F);
%% 绘制散点图图
clc
clear all
% 画折线图 点坐标，点顺序 xlabel ylabel title color LineStyle legend（图例）
% hold on hold off figure（创建窗口）subplot（给窗口划分）
% 绘制散点图
N=100;
x=2*rand(1,N)-1;%1 行 N 列
y=2*rand(1,N)-1;
% scatter(x,y,'filled');%'filled' 表示填充
in = x.^2 +y.^2 <= 1;
out = ~in;
hold on
% scatter(x(in),y(in));
% scatter(x(out),y(out),'x');
plot(x(in),y(in),'linestyle','none','marker','o');
plot(x(out),y(out),'linestyle','none','marker','x');
theta =0:0.01:2*pi;
circle_x = cos(theta);
circle_y = sin(theta);
plot(circle_x,circle_y,'color','black','linewidth',2);
%% 绘制饼图
clc
clear all
%  饼的名字
labels = {'张三','李四','王五','赵六'};
% 饼的大小
X =[50 100 150 200];
% 哪一块饼要分离出来
explode = [0 0 0 1];
% 画饼图
pie(X,explode,labels);
title('本月业绩');
%% 图形句柄
t=0:0.01:10;
y=sin(t);
hd=plot(t,y);%得到的是线的句柄
% 通过set函数可以修改图形的属性
h = gcf ;%得到当前图像的句柄
%% 动画初步
clear
clc
t = -pi:0.01:pi;
h = plot(0,0);
 axis([-4,4,-1,1]);
%  每过1/25秒刷新一次数据
fs = 1/25;
% 方法一：使用pause函数，但是动画的时候不能操作
% for i = 1:5:length(t)
%      set(h, 'xdata' , t(1:i), 'ydata', sin(t(1:i)));%更新句柄的数据
%      pause(fs);%绘制期间不能进行其他操作
% end

%方法二、采用定时器
global i
i=1;
timer1=timer('Period',fs,'TimerFcn',{@callback,h,t},'ExecutionMode','fixedSpacing');
start(timer1);
%% 蒙特卡洛法求PI动画
clear
clc
R=10;
N=10;
fs = 0.8;

global X Y num
X=[];
Y=[];
num =0;
t=0:0.01:2*pi;
circle_x = R*cos(t);
circle_y = R*sin(t);
plot(circle_x,circle_y,'color','k');
axis('square');
timer1=timer('Period',fs,'TimerFcn',{@callback,R,N},'ExecutionMode','fixedSpacing');
hold on
start(timer1);
%% 机器人工具箱；仿真训练
clear
clc
% 先画一个在原点的坐标系
T1 = SE2(1,2,30*pi/180);
axis([0 5 0 5]);
axis square;
hold on;
grid on;
 trplot2(T1,'frame','1','color','b');
 
 T2 = SE2(2,1,0);
trplot2(T2,'frame','2','color','r');

% 左乘 和 右乘的区分
T3 = T1*T2; %先进行T1变换，再进行T2变换
trplot2(T3,'frame','3','color','g');

T4 = T2*T1; %先进行T2变换，再进行T1变换
trplot2(T4,'frame','4','color','c');

P = [3;2];
plot_point(P,'*');
P1 = double(inv(T1)) * [P; 1]; %得到P在坐标系1的位姿
h2e (P1);%h表示齐次形式；e表示欧几里得点 e2h
%  homtrans(double(inv(T1)),P); %理解为其次变换的函数 更简洁的表达式

%  三维空间描述
 R = rotx(pi/2);%表示绕x轴旋转pi/2
 trplot(R);
 tranimate(R);%制作旋转动画,将世界坐标系旋转到指定坐标系过程
 R = rotx(pi/2)*roty(pi/2);%世界坐标系先绕x轴转动90°再绕y轴转动90°；
 
%  欧拉角表示方法 欧拉角表示的绕一个特定的轴旋转两次，但不是重复旋转
% ZYZ式
R = rotz(0.1)*roty(0.2)*rotz(0.3);
R = eul2r(0.1,0.2,0.3);%上式与该式的结果相同
% 逆解问题：找到给定旋转矩阵的欧拉角,反函数的关系
gamma = tr2eul(R);
% 中间有反解，还有奇异值的问题，我还没搞清楚
% 奇异点；旋转轴相互平行
% 需要仔细看

% RPY角 卡尔丹角；对于航空和车辆而言，x轴为前进方向，z轴竖直向下，y轴指向右手方向
% Roll:横滚 Pitch: 俯仰 Yaw: 偏航(航向) 
R = rpy2r(0.1,0.2,0.3);
gamma = tr2rpy(R);
%% 毕设仿真部分 2D 输入角度，确定杆长
% 先画一个坐标系{O}
clear 
clc
T0 =SE2(0,0,0);
trplot(T0,'frame','O');
hold on;
grid on;
% 确定坐标系 C
theta =-pi/6;
L =3;
axis([-(L+2) L -(L+2) L]);
% T = SE2(0,0,pi/2);
C = [0 ;-L ];
B = [1/3*C,2/3*C];%我的B的坐标应该相对C坐标系而言的
A =[-1 1;1 -1];
R = double(SE2(0,0,theta));
C2 =homtrans(R,C);
B2 =homtrans(R,B);
Tc=SE2(C2(1),C2(2),theta);


trplot(Tc,'frame','C');
plot_point(B2,'*');
plot_point(C2,'*');
plot_point(A,'o');

P =[[0;0] B2  C2 A];
point_n=length(P);
Link=zeros(point_n,point_n);
for i=1:1:point_n/2
    Link(i,point_n/2+i)=1;
    Link(point_n/2+i,i)=1;
end
gplot(Link,P');
% line([A(1,1) B2(1,1)],[A(2,1) B2(2,1)]);
% line([A(1,2) B2(1,2)],[A(2,2) B2(2,2)]);
% line([0 C2(1,1)],[0 C2(2,1)]);

hold off;
ML =B2-A;
L1=norm(ML(:,1))
L2=norm(ML(:,2))
%% 3D 仿真 对于3维我们要想象一个地球仪
% 还没有限位 所以无法建立工作空间 一个alpha beta gamma决定
% 还有实际能够到达的空间
clear 
clc
% 变量声明
alpha =0; %前后伸展 正
beta =pi/4;%沿颈肩上下伸展 负
gamma =0;% 负
L = 10;
H =8;
C0 = [0;0;-L];
A1 = [0 5 H];
F = [-10 0 H-3];
%F A2 A3 A4 A5...在坐标系O的表示
A =[F        ;
        0 5 H;
        0 5 H;
        0 -5 H;
        0 -5 H];
 %B1 B2 B3 B4 B5 在坐标系C的表示
cB = [2 3 -2;
          1 2 2;
          2 -2 -2;
          1 -2 2;
          -2 0 -2 ];
dalpha = pi/200;
for k=1:1:50
alpha = alpha +dalpha;
% 计算部分
cBT=cB';
AT=A';
% R = rpy2r(alpha,beta,gamma);
R = trotz(gamma)*troty(beta)*trotx(alpha);
C1 = homtrans(R,C0);%旋转顺序 Z，Y，X
T0=[ eye(3,3) ,zeros(3,1);zeros(1,3),1];
% T1表示中心的位姿 即 AcT
T1 = transl(C1')*trotz(gamma)*troty(beta)*trotx(alpha);
BT = homtrans(T1,cBT);
 
%  计算绳线的长度 A1B1
ML = AT-BT;
stringL=zeros(1,5);
    for i=1:1:5
        stringL(1,i)=norm(ML(:,i));
    end
stringL
%  画图部分
trplot(T0,'frame','O','color','r');
axis([-1.2*L 1.2*L -1.2*L 1.2*L  -1.2*L 1.2*L ]);
hold on
trplot(T1,'frame','C','color','b');
 plot3(AT(1,:)',AT(2,:)',AT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','g');
 plot3(BT(1,:)',BT(2,:)',BT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','g');
%  进行连线 Ai 和Bi连线
ABT =[AT BT];
point_nAB=length(ABT);
LinkAB=zeros(point_nAB,point_nAB);
for i=1:1:point_nAB/2
    LinkAB(i,point_nAB/2+i)=1;
    LinkAB(point_nAB/2+i,i)=1;
end
gplot23D(LinkAB,ABT');
% 绘制Bi之间的连线
LinkB = ones(5,5);
LinkB = LinkB  -eye(5,5);
LinkB(4,1)=0;
LinkB(1,4)=0;
gplot23D(LinkB,BT');
% 连接OC两点
OC =[[0;0;0] C1];
plot3(OC(1,:)',OC(2,:)',OC(3,:)','color','k','LineWidth',4);
%连接 A1F两点
A1F=[A1' F'];
plot3(A1F(1,:)',A1F(2,:)',A1F(3,:)');
hold off
view([-1,-1.3,0.5]);
drawnow();
hold off
view([-1,-1.3,0.5]);
end 
% % 提取矩阵T中的旋转部分
% R2=t2r(T1);
% % 提取矩阵T中的平移部分
% trans1(T)'
% % 反解得到姿态角 ; NaN not a num 
% gamma=tr2rpy(R2);

%% 控制轨迹生成
% 还没有限位 所以无法建立工作空间 一个alpha beta gamma决定
clear 
clc
% 变量声明
alpha =0; %前后伸展 正
beta =pi/4;%沿颈肩上下伸展 负
gamma =0;% 负
L = 10;
H =8;
C0 = [0;0;-L];
A1 = [0 5 H];
F = [-10 0 H-3];
%F A2 A3 A4 A5...在坐标系O的表示 A1 A4 A5 一边 A2 A3一边 
A =[F        ;
        0 -5 H;
        0 -5 H;
        0 5 H;
        0 5 H];
 %B1 B2 B3 B4 B5 在坐标系C的表示，
 %移动平台底面的3个点为 B1 B3 B5
cB = [-2 0 -2;
          1 -2 2;
          2 -2 -2;
          1 2 2;
          2 3 -2 ];%2 3 -2
Tspan=50;
dalpha = pi/4/Tspan;
stringL=zeros(Tspan,5);
for k=1:1:Tspan
alpha = alpha +dalpha;
% 计算部分
cBT=cB';
AT=A';
% R = rpy2r(alpha,beta,gamma);
R = trotz(gamma)*troty(beta)*trotx(alpha);
C1 = homtrans(R,C0);%旋转顺序 Z，Y，X
T0=[ eye(3,3) ,zeros(3,1);zeros(1,3),1];
% T1表示中心的位姿 即 AcT
T1 = transl(C1')*trotz(gamma)*troty(beta)*trotx(alpha);
BT = homtrans(T1,cBT);
%  计算绳线的长度 A1B1
ML = AT-BT;
for i=1:1:5
    stringL(k,i)=norm(ML(:,i));
end
%  画图部分
trplot(T0,'frame','O','color','r');
axis([-1.2*L 1.2*L -1.2*L 1.2*L  -1.2*L 1.2*L ]);
hold on
trplot(T1,'frame','C','color','b');
 plot3(AT(1,:)',AT(2,:)',AT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','g');
 plot3(BT(1,:)',BT(2,:)',BT(3,:)','o','color','b','MarkerSize',10,'MarkerFaceColor','g');
%  进行连线 Ai 和Bi连线
ABT =[AT BT];
point_nAB=length(ABT);
LinkAB=zeros(point_nAB,point_nAB);
for i=1:1:point_nAB/2
    LinkAB(i,point_nAB/2+i)=1;
    LinkAB(point_nAB/2+i,i)=1;
end
gplot23D(LinkAB,ABT');
% 绘制Bi之间的连线
LinkB = ones(5,5);
LinkB = LinkB  -eye(5,5);
LinkB(3,4)=0;%1 4
LinkB(4,3)=0;
gplot23D(LinkB,BT');
% 连接OC两点
OC =[[0;0;0] C1];
plot3(OC(1,:)',OC(2,:)',OC(3,:)','color','k','LineWidth',4);
%连接 A1F两点
A1F=[A1' F'];
plot3(A1F(1,:)',A1F(2,:)',A1F(3,:)');
hold off
view([-1,-1.3,0.5]);
drawnow();
hold off
view([-1,-1.3,0.5]);
end 

stringL
t=1:1:Tspan;
figure(100),plot(t,stringL);title('各绳线长度随时间变化图');
motorR = 2;
%以初始状态的长度为基准
motorTheta=zeros(Tspan,5);
for i=1:1:Tspan
    motorTheta(i,:)=(stringL(i,:)-stringL(1,:))/2*180/pi;
end
motorTheta
figure(200),plot(t,motorTheta);title('各舵机转角随时间变化图');
xlswrite('trajactory.xls',[motorTheta;stringL]);








