clear;
clc;
%用Link构建三个连杆L1，L2，L3，并将对应的参数代入
%创建连杆L1
% L0 = Link('d', 0, 'a', 0,'alpha', pi/2);
% L1 = Link('d', 0, 'a', 0.8, 'alpha', -pi/2);
% %创建连杆L2
% L2 = Link('d', 0, 'a', 1, 'alpha',pi/2);
% %创建连杆L3
% L3 = Link('d', 0, 'a', 1.2,'alpha', 0);

L0 = Link('d', 0, 'a', 0,'alpha', pi/2);
L1 = Link('d', 0, 'a', 0.8, 'alpha', -pi/2);
%创建连杆L2
L2 = Link('d', 0, 'a', 1, 'alpha',pi/2);
% %创建连杆L3
L3 = Link('d', 0, 'a', 1.2,'alpha', 0);

%根据上面的三个连杆创建三自由度的机械臂的模型
% robot=SerialLink([L1,L2,L3,L4,L5,L6]);
robot=SerialLink([L0,L1,L2,L3]);
robot.name ='STU2022';%将整个机械臂模型命名
robot.comment = 'STU2022';%给机械臂添加注释  
robot.display();%输出机械臂D-H参数表
% theta = [0,0,0,0,0,0];
theta = [0,0,0,0];
%imshow(img); 
robot.plot(theta,'workspace',[-5 5 -5 5 -5 5],'tilesize',10); 
%将起始点和终止点分别存在两个平移变换的齐次变换矩阵T1、T2中
%起始点可以由上面的参数算得为（3，0，0），终止点可以定在一定的允许范围内
%写入的参数为 x   y   z
T0 = transl( 0 ,  -3 ,  0 );
T1 = transl( 1,    1 ,   1 );


%用ikine函数可以实现机器人运动学逆问题的求解,分别对T1和T2变换矩阵求解
%而且当反解的机器人对象的自由度少于6时，要用mask vector减少自由度
%1代表要的自由度，0代表不要的自由度，所以'mask',[1 1 1 0 0 0]表示只取三个自由度
q1 = robot.ikine(T0,'mask',[1 1 1 0 0 0]);
q2 = robot.ikine(T1,'mask',[1 1 1 0 0 0]);

%jtraj函数用于进行机械臂的关节空间轨迹规划
%并且用上面反解求出的的q1作为状态1，q2作为状态2，
%而且jtraj函数括号内第三个位置填的是规划轨迹长度放在x轴上的起始位置，以及分度值，终点位置
%0:0.01:1则是分为了101个点
B=jtraj(q1,q2,0:0.01:1);
%fkine函数可以实现机器人运动学正问题的求解，即求正解
%并将该正解代入一个平移变换的齐次变换矩阵中
JTA=transl(robot.fkine(B));
%用plot2函数显示用正向求解得出的轨迹，即显示上面的JTA齐次变换矩阵表示的结果
plot2(JTA);
hold on;
%用plot函数显示出整个机械臂的运动
robot.plot(B);

clear
%% 初始化x,y
x = [17.447 17.470 17.534 17.565 17.635]/5;
y = [sin(deg2rad(5/2))^2 sin(deg2rad(10/2))^2 sin(deg2rad(15/2))^2 sin(deg2rad(20/2))^2 sin(deg2rad(25/2))^2];
%% 最小二乘法计算a,b
xp = [x; ones(size(x))];
C = y * xp' * inv(xp*xp');
a = C(1);
b = C(2);
%% 作图
plot(x, y, 'o')
hold on
plot(x, a*x+b, '--r');
%% 验证最小平方和
fun = @(D) sum((D(1)*x+D(2)-y).^2);
D0 = [0 0];
D = fminsearch(fun, D0);
a_ = D(1);
b_ = D(2);




