clear;

%创建连杆伪连杆L0，以及连杆L1,L2,L3
L0 = Link('d', 0, 'a', 0,'alpha', pi/2);
L1 = Link('d', 0, 'a', 0.8, 'alpha', -pi/2);
L2 = Link('d', 0, 'a', 1, 'alpha',pi/2);
L3 = Link('d', 0, 'a', 1.2,'alpha', 0);

%根据上面的三个连杆创建三自由度的机械臂的模型
% robot=SerialLink([L1,L2,L3,L4,L5,L6]);
robot=SerialLink([L0,L1,L2,L3]);
robot.name ='STU2022';
robot.comment = 'STU2022';
robot.display();%输出机械臂D-H参数表
theta = [0,0,0,0];
robot.plot(theta,'workspace',[-5 5 -5 5 -5 5],'tilesize',10); 


T0 = transl( 0 ,  -3 ,  0 );%起始坐标
T1 = transl( 1,    1 ,   1 );%终点坐标


%用ikine函数可以实现机器人运动学逆问题的求解,分别对T1和T2变换矩阵求解
%而且当反解的机器人对象的自由度少于6时，要用mask vector减少自由度
%1代表要的自由度，0代表不要的自由度，所以'mask',[0 1 1 1 0 0]表示只取三个自由度，其中丢掉伪连杆的自由度
q1 = robot.ikine(T0,'mask',[0 1 1 1 0 0]);
q2 = robot.ikine(T1,'mask',[0 1 1 1 0 0]);

%jtraj函数用于进行机械臂的关节空间轨迹规划
B=jtraj(q1,q2,0:0.01:1);

%fkine函数可以实现机器人运动学正问题的求解，即求正解
JTA=transl(robot.fkine(B));
%用plot2函数输出用正向求解得出的轨迹
plot2(JTA);
hold on;
%用plot函数显示出整个机械臂的运动
robot.plot(B);

clear

