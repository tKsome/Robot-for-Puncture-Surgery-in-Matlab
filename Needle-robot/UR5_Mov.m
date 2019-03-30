close all;
clear;
global thi thf tb tf m l v b Jm Dm Ra La Kt Ke N g 
global Link
%My_fkine：正向运动学解
%输入：角度值（非弧度值）
%输出：T06变换矩阵
%My_ikine：逆向运动学求逆解析解（仅UR5 DH模型8组解）
%输入：变换矩阵
%输出：弧度值（非角度值）
%Mov_One_Step：机械臂一步动作
%输入：角度值（非弧度值）
%simout输出弧度值
%%%%%%%%%%%%%%%%%%%%%%%%%%%%参数初始化%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1); 
UR5_Mdl;
ToDeg = 180/pi;%Trance the value of angle
ToRad = pi/180;
th1=0;%+180°…… -180°
th2=-90;%+160°…… -160°
th3=0;%+160°…… -160°
th4=0;%+180°…… -180°
th5=0;%+180°…… -180°
dz6=0;
th7=0;%+180°…… -180°
stp=10;%initial 机械臂正立姿态
r=100;%圆柱面半径
hand_len=100;
tb=1;%轨迹规划匀速时间
tf=4;%目标时间
joint2joint_length=[Link(2).dz -Link(3).dx -Link(4).dx Link(5).dz Link(6).dz  1 Link(7).dz];
joint2joint_length=joint2joint_length/1000;%连杆长度
% dtime=0.02;
theta=[th1 th2 th3 th4 th5 dz6 th7];
theta_before = theta;
dyna_type=1;%运动姿势序号，UR5--6自由度8种解析解姿势
Mov_One_Step(th1,th2,th3,th4,th5,dz6,th7,0); 
hold on;
Forward_Value=My_fkine(theta_before);
Over_traj0=[];%无插值末端轨迹
Over_traj1=[];%抛物线线性插值末端轨迹
Over_traj2=[];%三次多项式插值末端轨迹
Over_traj3=[];%五次多项式插值末端轨迹
% pause;
%%%%%%%%%%%%%%%%%%%%%%%%穿刺手术机器人%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%参数初始化
needle_woking=[5 10 15 20 25 30 35 40 45 50 45 40 35 30 25 20 15 10 5 0];%工作演示长度，关节6伸缩
y_label=-300;
x_label=-200;
z_label=0;%环境坐标
[body,len]=body(x_label,y_label,z_label);%定位body坐标
plot3(body(:,1),body(:,2),body(:,3));
hold on;
% %二维主手端标点
figure(2);
plot(body(:,2),body(:,1));
[y_body,x_body] = ginput(1);
% %通过求法向量计算角度
xlabell=x_body;
ylabell=y_body;
zlabell=sqrt(r^2-(abs(y_label)-abs(ylabell))^2);%目标坐标值
sourcex=x_body;
sourcey=y_label;
sourcez=z_label;
needle_direction=rad2deg(atan2((zlabell-sourcez),(ylabell-sourcey)));
% %目标角度转化成目标旋转矩阵
Trance_initial=[0 1 0;1 0 0;0 0 -1];
if needle_direction>=90
    flag=1;
    needle_direction=180-needle_direction;
    needle_direction=90-needle_direction;
    trance_theta=needle_direction;
    needle_direction=-needle_direction;%需要x坐标正转的角度
else
    flag=0;
    needle_direction=90-needle_direction;%需要x坐标反转的角度
    trance_theta=needle_direction;
end
trance_theta=ToRad*trance_theta;%Z点变换值
needle_direction=ToRad*needle_direction;%角度转弧度
Trance_twostep=[1 0 0;
    0 cos(needle_direction) sin(needle_direction);
    0 -sin(needle_direction) cos(needle_direction)];
Trance_rot=Trance_twostep*Trance_initial;
zb=hand_len*cos(trance_theta);
yb=hand_len*sin(trance_theta);
if flag==1
    yb=-yb;
end    
Trance_Value=[Trance_rot(1,:) xlabell;
    Trance_rot(2,:) ylabell+yb;
    Trance_rot(3,:) zlabell+zb;
    0 0 0 1];
% %到达目标点进行穿刺手术
%运动规划0，给定目标点，通过角度规划直接到达，无插值%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(1);
theta_now=theta_before;
Inverse_Value=My_ikine(Trance_Value);
theta_before=Inverse_Value(dyna_type,:)*ToDeg;
theta_before=[theta_before(1:5),hand_len,theta_before(5)];
for traj_i=1:1:7
    a=My_traj(theta_now(traj_i),theta_before(traj_i),tb,tf,1,joint2joint_length(traj_i),10,0,1e-5,1e-6,10,4.4e-3,0.05,0.05,5,0);
    sim('Robot_Joint_Prototype11',tf);
    theta_traj(:,traj_i)=simout_theta;%输出为弧度值
    vol(:,traj_i)=simout_vol;
    acc(:,traj_i)=simout_acc;
end
theta_traj=theta_traj*ToDeg;%转角度值
theta_now=theta_before;
for n=1:1:length(theta_traj)
    if n==length(theta_traj)
        Mov_One_Step(theta_traj(n,1),theta_traj(n,2),theta_traj(n,3),theta_traj(n,4),theta_traj(n,5),theta_traj(n,6),theta_traj(n,7),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(theta_traj(n,1),theta_traj(n,2),theta_traj(n,3),theta_traj(n,4),theta_traj(n,5),theta_traj(n,6),theta_traj(n,7),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
    mid=My_fkine(theta_traj(n,:));
    Over_traj0(n,1)=mid(1,4);
    Over_traj0(n,2)=mid(2,4);
    Over_traj0(n,3)=mid(3,4);
end
for n=1:1:length(needle_woking)
    if n==length(needle_woking)
        Mov_One_Step(theta_traj(end,1),theta_traj(end,2),theta_traj(end,3),theta_traj(end,4),theta_traj(end,5),theta_traj(end,6)+needle_woking(1,n),theta_traj(end,7),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(theta_traj(end,1),theta_traj(end,2),theta_traj(end,3),theta_traj(end,4),theta_traj(end,5),theta_traj(end,6)+needle_woking(1,n),theta_traj(end,7),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
end
% pause;
%复原
theta_before = theta;
for traj_i=1:1:7
    a=My_traj(theta_now(traj_i),theta_before(traj_i),tb,tf,1,joint2joint_length(traj_i),10,0,1e-5,1e-6,10,4.4e-3,0.05,0.05,5,0);
    sim('Robot_Joint_Prototype11',tf);
    theta_traj(:,traj_i)=simout_theta;%输出为弧度值
    vol(:,traj_i)=simout_vol;
    acc(:,traj_i)=simout_acc;
end
theta_traj=theta_traj*ToDeg;%转角度值
theta_now=theta_before;
for n=1:1:length(theta_traj)
    if n==length(theta_traj)
        Mov_One_Step(theta_traj(n,1),theta_traj(n,2),theta_traj(n,3),theta_traj(n,4),theta_traj(n,5),theta_traj(n,6),theta_traj(n,7),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(theta_traj(n,1),theta_traj(n,2),theta_traj(n,3),theta_traj(n,4),theta_traj(n,5),theta_traj(n,6),theta_traj(n,7),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
end
pause(2)
% % %运动规划1，避免穿透和碰撞,机械臂先运动到人体手术区域正上空并完成末端关节的角度转换,一次插值,抛物线过度方法
plan1_x=x_label;
plan1_y=y_label;
plan1_z=400;
Trance_plan1=[Trance_Value(1,1:3) plan1_x;
    Trance_Value(2,1:3) plan1_y+yb;
    Trance_Value(3,1:3) plan1_z+zb;
    0 0 0 1];
figure(1);
theta_now=theta_before;
Inverse_Value=My_ikine(Trance_plan1);
theta_before=Inverse_Value(dyna_type,:)*ToDeg;
theta_before=[theta_before(1:5),hand_len,theta_before(5)];
for traj_i=1:1:7
    a=My_traj(theta_now(traj_i),theta_before(traj_i),tb,tf,1,joint2joint_length(traj_i),10,0,1e-5,1e-6,10,4.4e-3,0.05,0.05,5,0);
    sim('Robot_Joint_Prototype11',tf);
    theta_traj(:,traj_i)=simout_theta;
    vol(:,traj_i)=simout_vol;
    acc(:,traj_i)=simout_acc;%输出为弧度值
end
theta_traj1=theta_traj*ToDeg;%转角度值
vol1=vol;
acc1=acc;
%到达目标点
theta_now=theta_before;
Inverse_Value=My_ikine(Trance_Value);
theta_before=Inverse_Value(dyna_type,:)*ToDeg;
theta_before=[theta_before(1:5),hand_len,theta_before(5)];
for traj_i=1:1:7
    a=My_traj(theta_now(traj_i),theta_before(traj_i),tb,tf,1,joint2joint_length(traj_i),10,0,1e-5,1e-6,10,4.4e-3,0.05,0.05,5,0);
    sim('Robot_Joint_Prototype11',tf);
    theta_traj(:,traj_i)=simout_theta;%输出为弧度值
    vol(:,traj_i)=simout_vol;
    acc(:,traj_i)=simout_acc;
end
theta_traj=theta_traj*ToDeg;%转角度值
theta_now=theta_before;
theta_traj=[theta_traj1;theta_traj];
vol=[vol1;vol];
acc=[acc1;acc];
for n=1:1:length(theta_traj)
    if n==length(theta_traj)
        Mov_One_Step(theta_traj(n,1),theta_traj(n,2),theta_traj(n,3),theta_traj(n,4),theta_traj(n,5),theta_traj(n,6),theta_traj(n,7),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(theta_traj(n,1),theta_traj(n,2),theta_traj(n,3),theta_traj(n,4),theta_traj(n,5),theta_traj(n,6),theta_traj(n,7),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
    mid1=My_fkine(theta_traj(n,:));
    Over_traj1(n,1)=mid1(1,4);
    Over_traj1(n,2)=mid1(2,4);
    Over_traj1(n,3)=mid1(3,4);
end
for n=1:1:length(needle_woking)
    if n==length(needle_woking)
        Mov_One_Step(theta_traj(end,1),theta_traj(end,2),theta_traj(end,3),theta_traj(end,4),theta_traj(end,5),theta_traj(end,6)+needle_woking(1,n),theta_traj(end,7),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(theta_traj(end,1),theta_traj(end,2),theta_traj(end,3),theta_traj(end,4),theta_traj(end,5),theta_traj(end,6)+needle_woking(1,n),theta_traj(end,7),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
end
%复原，每次复原方法按原方法复原
theta_now=theta_before;
Inverse_Value=My_ikine(Trance_plan1);
theta_before=Inverse_Value(dyna_type,:)*ToDeg;
theta_before=[theta_before(1:5),0,theta_before(5)];
for traj_i=1:1:7
    a=My_traj(theta_now(traj_i),theta_before(traj_i),tb,tf,1,joint2joint_length(traj_i),10,0,1e-5,1e-6,10,4.4e-3,0.05,0.05,5,0);
    sim('Robot_Joint_Prototype11',tf);
    theta_traj12(:,traj_i)=simout_theta;
end
theta_traj12=theta_traj12*ToDeg;%转角度值
theta_now=theta_before;
theta_before = theta;
for traj_i=1:1:7
    a=My_traj(theta_now(traj_i),theta_before(traj_i),tb,tf,1,joint2joint_length(traj_i),10,0,1e-5,1e-6,10,4.4e-3,0.05,0.05,5,0);
    sim('Robot_Joint_Prototype11',tf);
    theta_traj112(:,traj_i)=simout_theta;%输出为弧度值
end
theta_traj112=theta_traj112*ToDeg;%转角度值
theta_now=theta_before;
theta_traj112=[theta_traj12;theta_traj112];
for n=1:1:length(theta_traj)
    if n==length(theta_traj)
        Mov_One_Step(theta_traj112(n,1),theta_traj112(n,2),theta_traj112(n,3),theta_traj112(n,4),theta_traj112(n,5),theta_traj112(n,6),theta_traj112(n,7),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(theta_traj112(n,1),theta_traj112(n,2),theta_traj112(n,3),theta_traj112(n,4),theta_traj112(n,5),theta_traj112(n,6),theta_traj112(n,7),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
end
pause(2)
% %运动规划二，三次多项式插值法%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v_mid=20;
plan1_x=x_label;
plan1_y=y_label;
plan1_z=400;
Trance_plan1=[Trance_Value(1,1:3) plan1_x;
    Trance_Value(2,1:3) plan1_y+yb;
    Trance_Value(3,1:3) plan1_z+zb;
    0 0 0 1];
theta_now=theta_before;%插值起始角度
Inverse_Value=My_ikine(Trance_plan1);%插值末端角度
theta_before=Inverse_Value(dyna_type,:)*ToDeg;
theta_before=[theta_before(1:5),hand_len,theta_before(5)];
for three_traj=1:1:7
q0=theta_now(1,three_traj);
q1=theta_before(1,three_traj);
t0=0;
t1=3;%终止时间
a0=q0;
v0=0;
v1=v_mid;%终止速度
a1=v0;
a2=(3/(t1)^2).*(q1-q0)-(1/t1).*(2.*v0+v1);
a3=(2/(t1)^3).*(q0-q1)+(1/t1^2).*(v0+v1);
t=t0:0.1:t1;
q31(three_traj,:)=a0+a1*t+a2*t.^2+a3*t.^3;
vv31(three_traj,:)=a1+2*a2*t+3*a3*t.^2;
a31(three_traj,:)=2*a2+6*a3*t;
end
theta_now=theta_before;
Inverse_Value=My_ikine(Trance_Value);
theta_before=Inverse_Value(dyna_type,:)*ToDeg;
theta_before=[theta_before(1:5),hand_len,theta_before(5)];
for three_traj=1:1:7
q0=theta_now(1,three_traj);
q1=theta_before(1,three_traj);
t0=0;
t1=3;%终止时间
a0=q0;
v0=v_mid;
v1=0;%终止速度
a1=v0;
a2=(3/(t1)^2).*(q1-q0)-(1/t1).*(2.*v0+v1);
a3=(2/(t1)^3).*(q0-q1)+(1/t1^2).*(v0+v1);
t=t0:0.1:t1;
q31(three_traj+7,:)=a0+a1*t+a2*t.^2+a3*t.^3;
vv31(three_traj+7,:)=a1+2*a2*t+3*a3*t.^2;
a31(three_traj+7,:)=2*a2+6*a3*t;
end
q31=[q31(1:7,:) q31(8:14,:)];
vv31=[vv31(1:7,:) vv31(8:14,:)];
a31=[a31(1:7,:) a31(8:14,:)];
for n=1:1:length(q31)
    if n==length(q31)
        Mov_One_Step(q31(1,n),q31(2,n),q31(3,n),q31(4,n),q31(5,n),q31(6,n),q31(7,n),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(q31(1,n),q31(2,n),q31(3,n),q31(4,n),q31(5,n),q31(6,n),q31(7,n),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
    mid2=My_fkine(q31(:,n)');
    Over_traj2(n,1)=mid2(1,4);
    Over_traj2(n,2)=mid2(2,4);
    Over_traj2(n,3)=mid2(3,4);
end
for n=1:1:length(needle_woking)
    if n==length(needle_woking)
        Mov_One_Step(theta_traj(end,1),theta_traj(end,2),theta_traj(end,3),theta_traj(end,4),theta_traj(end,5),theta_traj(end,6)+needle_woking(1,n),theta_traj(end,7),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(theta_traj(end,1),theta_traj(end,2),theta_traj(end,3),theta_traj(end,4),theta_traj(end,5),theta_traj(end,6)+needle_woking(1,n),theta_traj(end,7),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
end
%复原
theta_now=theta_before;
Inverse_Value=My_ikine(Trance_plan1);%插值末端角度
theta_before=Inverse_Value(dyna_type,:)*ToDeg;
theta_before=[theta_before(1:5),0,theta_before(5)];
for three_traj=1:1:7
q0=theta_now(1,three_traj);
q1=theta_before(1,three_traj);
t0=0;
t1=3;%终止时间
a0=q0;
v0=0;
v1=v_mid;%终止速度
a1=v0;
a2=(3/(t1)^2).*(q1-q0)-(1/t1).*(2.*v0+v1);
a3=(2/(t1)^3).*(q0-q1)+(1/t1^2).*(v0+v1);
t=t0:0.1:t1;
q33(three_traj,:)=a0+a1*t+a2*t.^2+a3*t.^3;
vv33(three_traj,:)=a1+2*a2*t+3*a3*t.^2;
a33(three_traj,:)=2*a2+6*a3*t;
end
theta_now=theta_before;
theta_before=theta;
for three_traj=1:1:7
q0=theta_now(1,three_traj);
q1=theta_before(1,three_traj);
t0=0;
t1=3;%终止时间
a0=q0;
v0=v_mid;
v1=0;%终止速度
a1=v0;
a2=(3/(t1)^2).*(q1-q0)-(1/t1).*(2.*v0+v1);
a3=(2/(t1)^3).*(q0-q1)+(1/t1^2).*(v0+v1);
t=t0:0.1:t1;
q33(three_traj+7,:)=a0+a1*t+a2*t.^2+a3*t.^3;
vv33(three_traj+7,:)=a1+2*a2*t+3*a3*t.^2;
a33(three_traj+7,:)=2*a2+6*a3*t;
end
q33=[q33(1:7,:) q33(8:14,:)];
vv33=[vv33(1:7,:) vv33(8:14,:)];
a33=[a33(1:7,:) a33(8:14,:)];
for n=1:1:length(q33)
    if n==length(q33)
        Mov_One_Step(q33(1,n),q33(2,n),q33(3,n),q33(4,n),q33(5,n),q33(6,n),q33(7,n),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(q33(1,n),q33(2,n),q33(3,n),q33(4,n),q33(5,n),q33(6,n),q33(7,n),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
end
pause(2)
%运动规划三，五次多项式插值，加速度保持连贯性%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
v_mid=20;
v_end=0;
a_end=0;%起始加速度与最终加速度
a_mid=-20;%插值加速度
plan1_x=x_label;
plan1_y=y_label;
plan1_z=400;
Trance_plan1=[Trance_Value(1,1:3) plan1_x;
    Trance_Value(2,1:3) plan1_y+yb;
    Trance_Value(3,1:3) plan1_z+zb;
    0 0 0 1];
theta_now=theta_before;%插值起始角度
Inverse_Value=My_ikine(Trance_plan1);%插值末端角度
theta_before=Inverse_Value(dyna_type,:)*ToDeg;
theta_before=[theta_before(1:5),hand_len,theta_before(5)];
theta_initial=theta_now;
theta_mid=theta_before;
theta_now=theta_before;
Inverse_Value=My_ikine(Trance_Value);
theta_before=Inverse_Value(dyna_type,:)*ToDeg;
theta_before=[theta_before(1:5),hand_len,theta_before(5)];
theta_end=theta_before;
for five_traj=1:1:7
    q_array=[theta_initial(1,five_traj),theta_mid(1,five_traj),theta_end(1,five_traj)];
    t_array=[0,3,6];%指定起止时间
    v_array=[0,v_mid,v_end];%指定起止速度
    a_array=[0,a_mid,a_end];%指定起止加速度
    t=[t_array(1)];
    q=[q_array(1)];
    vv=[v_array(1)];
    a=[a_array(1)];%初始状态
    for i=1:1:length(q_array)-1;
        T=3;
        a0=q_array(i);
        a1=v_array(i);
        a2=a_array(i)/2;
        a3=(20*q_array(i+1)-20*q_array(i)-(8*v_array(i+1)+12*v_array(i))*T-(3*a_array(i)-a_array(i+1))*T^2)/(2*T^3);
        a4=(30*q_array(i)-30*q_array(i+1)+(14*v_array(i+1)+16*v_array(i))*T+(3*a_array(i)-2*a_array(i+1))*T^2)/(2*T^4);
        a5=(12*q_array(i+1)-12*q_array(i)-(6*v_array(i+1)+6*v_array(i))*T-(a_array(i)-a_array(i+1))*T^2)/(2*T^5);%计算五次多项式系数 
        ti=t_array(i):0.1:t_array(i+1);
        qi=a0+a1*(ti-t_array(i))+a2*(ti-t_array(i)).^2+a3*(ti-t_array(i)).^3+a4*(ti-t_array(i)).^4+a5*(ti-t_array(i)).^5;
        vi=a1+2*a2*(ti-t_array(i))+3*a3*(ti-t_array(i)).^2+4*a4*(ti-t_array(i)).^3+5*a5*(ti-t_array(i)).^4;
        ai=2*a2+6*a3*(ti-t_array(i))+12*a4*(ti-t_array(i)).^2+20*a5*(ti-t_array(i)).^3;
        t=[t,ti(2:end)];q=[q,qi(2:end)];vv=[vv,vi(2:end)];a=[a,ai(2:end)];
    end
qq(five_traj,:)=q;
vvv(five_traj,:)=vv;
aa(five_traj,:)=a;
end
for n=1:1:length(qq)
    if n==length(qq)
        Mov_One_Step(qq(1,n),qq(2,n),qq(3,n),qq(4,n),qq(5,n),qq(6,n),qq(7,n),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(qq(1,n),qq(2,n),qq(3,n),qq(4,n),qq(5,n),qq(6,n),qq(7,n),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
    mid3=My_fkine(qq(:,n)');
    Over_traj3(n,1)=mid3(1,4);
    Over_traj3(n,2)=mid3(2,4);
    Over_traj3(n,3)=mid3(3,4);
end
for n=1:1:length(needle_woking)
    if n==length(needle_woking)
        Mov_One_Step(theta_traj(end,1),theta_traj(end,2),theta_traj(end,3),theta_traj(end,4),theta_traj(end,5),theta_traj(end,6)+needle_woking(1,n),theta_traj(end,7),0);
        plot3(body(:,1),body(:,2),body(:,3));
    else
    Mov_One_Step(theta_traj(end,1),theta_traj(end,2),theta_traj(end,3),theta_traj(end,4),theta_traj(end,5),theta_traj(end,6)+needle_woking(1,n),theta_traj(end,7),1);
    plot3(body(:,1),body(:,2),body(:,3));
    end
end
%画出末端轨迹比较
% figure(3);
% plot3(Over_traj0(:,1),Over_traj0(:,2),Over_traj0(:,3),'--r');grid on;hold on;
% plot3(Over_traj1(:,1),Over_traj1(:,2),Over_traj1(:,3),'-.g');grid on;hold on;
% plot3(Over_traj2(:,1),Over_traj2(:,2),Over_traj2(:,3),'*b');grid on;hold on;
% plot3(Over_traj3(:,1),Over_traj3(:,2),Over_traj3(:,3),'om');grid on;hold on;
% plot3(body(:,1),body(:,2),body(:,3));grid on;hold on;
% legend('--:无插值（红色）','-.抛物线线性插值（绿色）','*:三次多项式插值（蓝色）','o:五次多项式插值(紫色)');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%产生随机点，限制关节角度%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% N=100000;
% joint1=unifrnd(-180,180,[1,N]);%第一关节变量限位
% joint2=unifrnd(-160,160,[1,N]);%第二关节变量限位
% joint3=unifrnd(-160,160,[1,N]);%第三关节变量限位
% joint4=unifrnd(-180,180,[1,N]);%第四关节变量限位
% joint5=unifrnd(-180,180,[1,N]);%第五关节变量限位
% joint6=unifrnd(-10,180,[1,N]);%第六关节变量限位
% joint7=unifrnd(-180,180,[1,N]);
% G=cell(N,7);
% for n = 1:N
%     G{n}=[joint1(n) joint2(n) joint3(n) joint4(n) joint5(n) joint6(n) joint7(n)];
% end
% H1=cell2mat(G);
% for n = 1:N
%     T(:,:,n)=double(My_fkine(H1(n,:)));
% end
% scatter3(squeeze(T(1,4,:)),squeeze(T(2,4,:)),squeeze(T(3,4,:)))%随机点图