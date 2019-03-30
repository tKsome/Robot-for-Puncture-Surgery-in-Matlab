function [ikine_t]=My_ikine(Tbe)
%解析解
%Tbe:输入4阶变换矩阵
%仅限用于UR5-DH参数下的解析解求解方法
%    ti               di          ai-1        alphai-1 
global Link
MDH=[Link(2).th   Link(2).dz   Link(2).dx   Link(2).alf;
     Link(3).th   Link(3).dz   Link(3).dx   Link(3).alf;
     Link(4).th   Link(4).dz   Link(4).dx   Link(4).alf;
     Link(5).th   Link(5).dz   Link(5).dx   Link(5).alf;
     Link(6).th   Link(6).dz   Link(6).dx   Link(6).alf;
     Link(8).th   Link(8).dz   Link(8).dx   Link(8).alf];
 
nx=Tbe(1,1); ny=Tbe(2,1); nz=Tbe(3,1);
ox=Tbe(1,2); oy=Tbe(2,2); oz=Tbe(3,2);
ax=Tbe(1,3); ay=Tbe(2,3); az=Tbe(3,3);
px=Tbe(1,4); py=Tbe(2,4); pz=Tbe(3,4);
d1=MDH(1,2); d2=MDH(2,2); d3=MDH(3,2); d4=MDH(4,2); d5=MDH(5,2); d6=MDH(6,2);
a1=MDH(1,3); a2=MDH(2,3); a3=MDH(3,3); a4=MDH(4,3); a5=MDH(5,3); a6=MDH(6,3);
f1=MDH(1,4); f2=MDH(2,4); f3=MDH(3,4); f4=MDH(4,4); f6=MDH(5,4); f6=MDH(6,4); 
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              theta1
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta11=acos(d4/(sqrt((d6*ay-py)^2+(px-d6*ax)^2)+1e-6))+atan2((px-d6*ax),(d6*ay-py));
theta12=-acos(d4/(sqrt((d6*ay-py)^2+(px-d6*ax)^2)+1e-6))+atan2((px-d6*ax),(d6*ay-py));
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              theta5
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta51=acos(sin(theta11)*ax-cos(theta11)*ay);
theta52=-acos(sin(theta11)*ax-cos(theta11)*ay);
theta53=acos(sin(theta12)*ax-cos(theta12)*ay);
theta54=-acos(sin(theta12)*ax-cos(theta12)*ay);
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              theta6
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% theta61=atan2(-(sin(theta11)*ox-cos(theta11)*oy),(sin(theta11)*nx-cos(theta11)*ny));
% theta62=atan2(-(sin(theta12)*ox-cos(theta12)*oy),(sin(theta12)*nx-cos(theta12)*ny));
n61=nx*sin(theta11)-ny*cos(theta11);
m61=ox*sin(theta11)-oy*cos(theta11);
n62=nx*sin(theta12)-ny*cos(theta12);
m62=ox*sin(theta12)-oy*cos(theta12);
theta61=atan2(real(-m61/(1e-6+sin(theta51))),real(n61/(1e-6+sin(theta51))));
theta62=atan2(real(-m61/(1e-6+sin(theta52))),real(n61/(1e-6+sin(theta52))));
% theta63=atan2(-m61/sin(theta53),n61/sin(theta53));
% theta64=atan2(-m61/sin(theta54),n61/sin(theta54));
theta63=atan2(real(-m62/(1e-6+sin(theta51))),real(n62/(1e-6+sin(theta51))));
theta64=atan2(real(-m62/(1e-6+sin(theta52))),real(n62/(1e-6+sin(theta52))));
% theta67=atan2(-m62/sin(theta53),n62/sin(theta53));
% theta68=atan2(-m62/sin(theta54),n62/sin(theta54));
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              theta3
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
m31=d5*((cos(theta11)*nx+sin(theta11)*ny)*sin(theta61)+(cos(theta11)*ox+sin(theta11)*oy)*cos(theta61))-d6*(cos(theta11)*ax+sin(theta11)*ay)+cos(theta11)*px+sin(theta11)*py;
n31=d5*(nz*sin(theta61)+oz*cos(theta61))-d6*az+pz-d1;%theta11 and theta61
m33=d5*((cos(theta11)*nx+sin(theta11)*ny)*sin(theta62)+(cos(theta11)*ox+sin(theta11)*oy)*cos(theta62))-d6*(cos(theta11)*ax+sin(theta11)*ay)+cos(theta11)*px+sin(theta11)*py;
n33=d5*(nz*sin(theta62)+oz*cos(theta62))-d6*az+pz-d1;%theta11 and theta62
m35=d5*((cos(theta11)*nx+sin(theta11)*ny)*sin(theta63)+(cos(theta11)*ox+sin(theta11)*oy)*cos(theta63))-d6*(cos(theta11)*ax+sin(theta11)*ay)+cos(theta11)*px+sin(theta11)*py;
n35=d5*(nz*sin(theta63)+oz*cos(theta63))-d6*az+pz-d1;%theta11 and theta63
m36=d5*((cos(theta11)*nx+sin(theta11)*ny)*sin(theta64)+(cos(theta11)*ox+sin(theta11)*oy)*cos(theta64))-d6*(cos(theta11)*ax+sin(theta11)*ay)+cos(theta11)*px+sin(theta11)*py;
n36=d5*(nz*sin(theta64)+oz*cos(theta64))-d6*az+pz-d1;%theta11 and theta64
m32=d5*((cos(theta12)*nx+sin(theta12)*ny)*sin(theta61)+(cos(theta12)*ox+sin(theta12)*oy)*cos(theta61))-d6*(cos(theta12)*ax+sin(theta12)*ay)+cos(theta12)*px+sin(theta12)*py;
n32=d5*(nz*sin(theta61)+oz*cos(theta61))-d6*az+pz-d1;%theta12 and theta61
m34=d5*((cos(theta12)*nx+sin(theta12)*ny)*sin(theta62)+(cos(theta12)*ox+sin(theta12)*oy)*cos(theta62))-d6*(cos(theta12)*ax+sin(theta12)*ay)+cos(theta12)*px+sin(theta12)*py;
n34=d5*(nz*sin(theta62)+oz*cos(theta62))-d6*az+pz-d1;%theta12 and theta62
m37=d5*((cos(theta12)*nx+sin(theta12)*ny)*sin(theta63)+(cos(theta12)*ox+sin(theta12)*oy)*cos(theta63))-d6*(cos(theta12)*ax+sin(theta12)*ay)+cos(theta12)*px+sin(theta12)*py;
n37=d5*(nz*sin(theta63)+oz*cos(theta63))-d6*az+pz-d1;%theta12 and theta63
m38=d5*((cos(theta12)*nx+sin(theta12)*ny)*sin(theta64)+(cos(theta12)*ox+sin(theta12)*oy)*cos(theta64))-d6*(cos(theta12)*ax+sin(theta12)*ay)+cos(theta12)*px+sin(theta12)*py;
n38=d5*(nz*sin(theta64)+oz*cos(theta64))-d6*az+pz-d1;%theta12 and theta64
theta31=acos((m31^2+n31^2-a2^2-a3^2)/((2*a2*a3)+1e-6));
theta32=-acos((m31^2+n31^2-a2^2-a3^2)/((2*a2*a3)+1e-6));
theta33=acos((m33^2+n33^2-a2^2-a3^2)/((2*a2*a3)+1e-6));
theta34=-acos((m33^2+n33^2-a2^2-a3^2)/((2*a2*a3)+1e-6));
theta35=acos((m32^2+n32^2-a2^2-a3^2)/((2*a2*a3)+1e-6));
theta36=-acos((m32^2+n32^2-a2^2-a3^2)/((2*a2*a3)+1e-6));
theta37=acos((m34^2+n34^2-a2^2-a3^2)/((2*a2*a3)+1e-6));
theta38=-acos((m34^2+n34^2-a2^2-a3^2)/((2*a2*a3)+1e-6));
% theta39=acos((m35^2+n35^2-a2^2-a3^2)/(2*a2*a3));
% theta310=-acos((m35^2+n35^2-a2^2-a3^2)/(2*a2*a3));
% theta311=acos((m36^2+n36^2-a2^2-a3^2)/(2*a2*a3));
% theta312=-acos((m36^2+n36^2-a2^2-a3^2)/(2*a2*a3));
% theta313=acos((m37^2+n37^2-a2^2-a3^2)/(2*a2*a3));
% theta314=-acos((m37^2+n37^2-a2^2-a3^2)/(2*a2*a3));
% theta315=acos((m38^2+n38^2-a2^2-a3^2)/(2*a2*a3));
% theta316=-acos((m38^2+n38^2-a2^2-a3^2)/(2*a2*a3));
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              theta2
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%nm31 theta31
theta21=atan2(real(n31*(a2+a3*cos(theta31))-m31*a3*sin(theta31)),real(m31*(a2+a3*cos(theta31))+n31*a3*sin(theta31)));
% theta22=atan2((n32*(a2+a3*cos(theta31))-m32*a3*sin(theta31)),(m32*(a2+a3*cos(theta31))+n32*a3*sin(theta31)));
% theta23=atan2((n33*(a2+a3*cos(theta31))-m33*a3*sin(theta31)),(m33*(a2+a3*cos(theta31))+n33*a3*sin(theta31)));
% theta24=atan2((n34*(a2+a3*cos(theta31))-m34*a3*sin(theta31)),(m34*(a2+a3*cos(theta31))+n34*a3*sin(theta31)));
% theta25=atan2((n35*(a2+a3*cos(theta31))-m35*a3*sin(theta31)),(m35*(a2+a3*cos(theta31))+n35*a3*sin(theta31)));
% theta26=atan2((n36*(a2+a3*cos(theta31))-m36*a3*sin(theta31)),(m36*(a2+a3*cos(theta31))+n36*a3*sin(theta31)));
% theta27=atan2((n37*(a2+a3*cos(theta31))-m37*a3*sin(theta31)),(m37*(a2+a3*cos(theta31))+n37*a3*sin(theta31)));
% theta28=atan2((n38*(a2+a3*cos(theta31))-m38*a3*sin(theta31)),(m38*(a2+a3*cos(theta31))+n38*a3*sin(theta31)));
%nm31 theta32
theta22=atan2(real(n31*(a2+a3*cos(theta32))-m31*a3*sin(theta32)),real(m31*(a2+a3*cos(theta32))+n31*a3*sin(theta32)));
% theta221=atan2((n32*(a2+a3*cos(theta32))-m32*a3*sin(theta32)),(m32*(a2+a3*cos(theta32))+n32*a3*sin(theta32)));
% theta231=atan2((n33*(a2+a3*cos(theta32))-m33*a3*sin(theta32)),(m33*(a2+a3*cos(theta32))+n33*a3*sin(theta32)));
% theta241=atan2((n34*(a2+a3*cos(theta32))-m34*a3*sin(theta32)),(m34*(a2+a3*cos(theta32))+n34*a3*sin(theta32)));
% theta251=atan2((n35*(a2+a3*cos(theta32))-m35*a3*sin(theta32)),(m35*(a2+a3*cos(theta32))+n35*a3*sin(theta32)));
% theta261=atan2((n36*(a2+a3*cos(theta32))-m36*a3*sin(theta32)),(m36*(a2+a3*cos(theta32))+n36*a3*sin(theta32)));
% theta271=atan2((n37*(a2+a3*cos(theta32))-m37*a3*sin(theta32)),(m37*(a2+a3*cos(theta32))+n37*a3*sin(theta32)));
% theta281=atan2((n38*(a2+a3*cos(theta32))-m38*a3*sin(theta32)),(m38*(a2+a3*cos(theta32))+n38*a3*sin(theta32)));
%nm37 theta35  已改 theta33<-->theta35
% theta212=atan2((n31*(a2+a3*cos(theta33))-m31*a3*sin(theta33)),(m31*(a2+a3*cos(theta33))+n31*a3*sin(theta33)));
% theta222=atan2((n32*(a2+a3*cos(theta33))-m32*a3*sin(theta33)),(m32*(a2+a3*cos(theta33))+n32*a3*sin(theta33)));
% theta232=atan2((n33*(a2+a3*cos(theta33))-m33*a3*sin(theta33)),(m33*(a2+a3*cos(theta33))+n33*a3*sin(theta33)));
% theta242=atan2((n34*(a2+a3*cos(theta33))-m34*a3*sin(theta33)),(m34*(a2+a3*cos(theta33))+n34*a3*sin(theta33)));
% theta252=atan2((n35*(a2+a3*cos(theta33))-m35*a3*sin(theta33)),(m35*(a2+a3*cos(theta33))+n35*a3*sin(theta33)));
% theta262=atan2((n36*(a2+a3*cos(theta33))-m36*a3*sin(theta33)),(m36*(a2+a3*cos(theta33))+n36*a3*sin(theta33)));
theta25=atan2(real(n37*(a2+a3*cos(theta35))-m37*a3*sin(theta35)),real(m37*(a2+a3*cos(theta35))+n37*a3*sin(theta35)));
% theta282=atan2((n38*(a2+a3*cos(theta33))-m38*a3*sin(theta33)),(m38*(a2+a3*cos(theta33))+n38*a3*sin(theta33)));
%nm37 theta34 已改 theta34<-->theta36
% theta213=atan2((n31*(a2+a3*cos(theta34))-m31*a3*sin(theta34)),(m31*(a2+a3*cos(theta34))+n31*a3*sin(theta34)));
% theta223=atan2((n32*(a2+a3*cos(theta34))-m32*a3*sin(theta34)),(m32*(a2+a3*cos(theta34))+n32*a3*sin(theta34)));
% theta233=atan2((n33*(a2+a3*cos(theta34))-m33*a3*sin(theta34)),(m33*(a2+a3*cos(theta34))+n33*a3*sin(theta34)));
% theta243=atan2((n34*(a2+a3*cos(theta34))-m34*a3*sin(theta34)),(m34*(a2+a3*cos(theta34))+n34*a3*sin(theta34)));
% theta253=atan2((n35*(a2+a3*cos(theta34))-m35*a3*sin(theta34)),(m35*(a2+a3*cos(theta34))+n35*a3*sin(theta34)));
% theta263=atan2((n36*(a2+a3*cos(theta34))-m36*a3*sin(theta34)),(m36*(a2+a3*cos(theta34))+n36*a3*sin(theta34)));
theta26=atan2(real(n37*(a2+a3*cos(theta36))-m37*a3*sin(theta36)),real(m37*(a2+a3*cos(theta36))+n37*a3*sin(theta36)));
% theta283=atan2((n38*(a2+a3*cos(theta34))-m38*a3*sin(theta34)),(m38*(a2+a3*cos(theta34))+n38*a3*sin(theta34)));
%nm33 theta35 已改 theta33<-->theta35
% theta214=atan2((n31*(a2+a3*cos(theta35))-m31*a3*sin(theta35)),(m31*(a2+a3*cos(theta35))+n31*a3*sin(theta35)));
% theta224=atan2((n32*(a2+a3*cos(theta35))-m32*a3*sin(theta35)),(m32*(a2+a3*cos(theta35))+n32*a3*sin(theta35)));
theta23=atan2(real(n33*(a2+a3*cos(theta33))-m33*a3*sin(theta33)),real(m33*(a2+a3*cos(theta33))+n33*a3*sin(theta33)));
% theta244=atan2((n34*(a2+a3*cos(theta35))-m34*a3*sin(theta35)),(m34*(a2+a3*cos(theta35))+n34*a3*sin(theta35)));
% theta254=atan2((n35*(a2+a3*cos(theta35))-m35*a3*sin(theta35)),(m35*(a2+a3*cos(theta35))+n35*a3*sin(theta35)));
% theta264=atan2((n36*(a2+a3*cos(theta35))-m36*a3*sin(theta35)),(m36*(a2+a3*cos(theta35))+n36*a3*sin(theta35)));
% theta274=atan2((n37*(a2+a3*cos(theta35))-m37*a3*sin(theta35)),(m37*(a2+a3*cos(theta35))+n37*a3*sin(theta35)));
% theta284=atan2((n38*(a2+a3*cos(theta35))-m38*a3*sin(theta35)),(m38*(a2+a3*cos(theta35))+n38*a3*sin(theta35)));
%nm33 theta36 已改 theta34<-->theta36
% theta215=atan2((n31*(a2+a3*cos(theta36))-m31*a3*sin(theta36)),(m31*(a2+a3*cos(theta36))+n31*a3*sin(theta36)));
% theta225=atan2((n32*(a2+a3*cos(theta36))-m32*a3*sin(theta36)),(m32*(a2+a3*cos(theta36))+n32*a3*sin(theta36)));
theta24=atan2(real(n33*(a2+a3*cos(theta34))-m33*a3*sin(theta34)),real(m33*(a2+a3*cos(theta34))+n33*a3*sin(theta34)));
% theta245=atan2((n34*(a2+a3*cos(theta36))-m34*a3*sin(theta36)),(m34*(a2+a3*cos(theta36))+n34*a3*sin(theta36)));
% theta255=atan2((n35*(a2+a3*cos(theta36))-m35*a3*sin(theta36)),(m35*(a2+a3*cos(theta36))+n35*a3*sin(theta36)));
% theta265=atan2((n36*(a2+a3*cos(theta36))-m36*a3*sin(theta36)),(m36*(a2+a3*cos(theta36))+n36*a3*sin(theta36)));
% theta275=atan2((n37*(a2+a3*cos(theta36))-m37*a3*sin(theta36)),(m37*(a2+a3*cos(theta36))+n37*a3*sin(theta36)));
% theta285=atan2((n38*(a2+a3*cos(theta36))-m38*a3*sin(theta36)),(m38*(a2+a3*cos(theta36))+n38*a3*sin(theta36)));
%nm38 theta37
% theta216=atan2((n31*(a2+a3*cos(theta37))-m31*a3*sin(theta37)),(m31*(a2+a3*cos(theta37))+n31*a3*sin(theta37)));
% theta226=atan2((n32*(a2+a3*cos(theta37))-m32*a3*sin(theta37)),(m32*(a2+a3*cos(theta37))+n32*a3*sin(theta37)));
% theta236=atan2((n33*(a2+a3*cos(theta37))-m33*a3*sin(theta37)),(m33*(a2+a3*cos(theta37))+n33*a3*sin(theta37)));
% theta246=atan2((n34*(a2+a3*cos(theta37))-m34*a3*sin(theta37)),(m34*(a2+a3*cos(theta37))+n34*a3*sin(theta37)));
% theta256=atan2((n35*(a2+a3*cos(theta37))-m35*a3*sin(theta37)),(m35*(a2+a3*cos(theta37))+n35*a3*sin(theta37)));
% theta266=atan2((n36*(a2+a3*cos(theta37))-m36*a3*sin(theta37)),(m36*(a2+a3*cos(theta37))+n36*a3*sin(theta37)));
% theta276=atan2((n37*(a2+a3*cos(theta37))-m37*a3*sin(theta37)),(m37*(a2+a3*cos(theta37))+n37*a3*sin(theta37)));
theta27=atan2(real(n38*(a2+a3*cos(theta37))-m38*a3*sin(theta37)),real(m38*(a2+a3*cos(theta37))+n38*a3*sin(theta37)));
%nm38 theta38
% theta217=atan2((n31*(a2+a3*cos(theta38))-m31*a3*sin(theta38)),(m31*(a2+a3*cos(theta38))+n31*a3*sin(theta38)));
% theta227=atan2((n32*(a2+a3*cos(theta38))-m32*a3*sin(theta38)),(m32*(a2+a3*cos(theta38))+n32*a3*sin(theta38)));
% theta237=atan2((n33*(a2+a3*cos(theta38))-m33*a3*sin(theta38)),(m33*(a2+a3*cos(theta38))+n33*a3*sin(theta38)));
% theta247=atan2((n34*(a2+a3*cos(theta38))-m34*a3*sin(theta38)),(m34*(a2+a3*cos(theta38))+n34*a3*sin(theta38)));
% theta257=atan2((n35*(a2+a3*cos(theta38))-m35*a3*sin(theta38)),(m35*(a2+a3*cos(theta38))+n35*a3*sin(theta38)));
% theta267=atan2((n36*(a2+a3*cos(theta38))-m36*a3*sin(theta38)),(m36*(a2+a3*cos(theta38))+n36*a3*sin(theta38)));
% theta277=atan2((n37*(a2+a3*cos(theta38))-m37*a3*sin(theta38)),(m37*(a2+a3*cos(theta38))+n37*a3*sin(theta38)));
theta28=atan2(real(n38*(a2+a3*cos(theta38))-m38*a3*sin(theta38)),real(m38*(a2+a3*cos(theta38))+n38*a3*sin(theta38)));
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%              theta4
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta41=atan2(real(nz*cos(theta61)-oz*sin(theta61))*cos(theta51)-az*sin(theta51),real(((cos(theta11)*nx+sin(theta11)*ny)*cos(theta61)-(cos(theta11)*ox+sin(theta11)*oy)*sin(theta61))*cos(theta51)-(cos(theta11)*ax+sin(theta11)*ay)*sin(theta51)))-theta21-theta31;
theta42=atan2(real(nz*cos(theta61)-oz*sin(theta61))*cos(theta51)-az*sin(theta51),real(((cos(theta11)*nx+sin(theta11)*ny)*cos(theta61)-(cos(theta11)*ox+sin(theta11)*oy)*sin(theta61))*cos(theta51)-(cos(theta11)*ax+sin(theta11)*ay)*sin(theta51)))-theta22-theta32;
theta43=atan2(real(nz*cos(theta62)-oz*sin(theta62))*cos(theta52)-az*sin(theta52),real(((cos(theta11)*nx+sin(theta11)*ny)*cos(theta62)-(cos(theta11)*ox+sin(theta11)*oy)*sin(theta62))*cos(theta52)-(cos(theta11)*ax+sin(theta11)*ay)*sin(theta52)))-theta23-theta33;
theta44=atan2(real(nz*cos(theta62)-oz*sin(theta62))*cos(theta52)-az*sin(theta52),real(((cos(theta11)*nx+sin(theta11)*ny)*cos(theta62)-(cos(theta11)*ox+sin(theta11)*oy)*sin(theta62))*cos(theta52)-(cos(theta11)*ax+sin(theta11)*ay)*sin(theta52)))-theta24-theta34;
theta45=atan2(real(nz*cos(theta63)-oz*sin(theta63))*cos(theta53)-az*sin(theta53),real(((cos(theta12)*nx+sin(theta12)*ny)*cos(theta63)-(cos(theta12)*ox+sin(theta12)*oy)*sin(theta63))*cos(theta53)-(cos(theta12)*ax+sin(theta12)*ay)*sin(theta53)))-theta25-theta35;
theta46=atan2(real(nz*cos(theta63)-oz*sin(theta63))*cos(theta53)-az*sin(theta53),real(((cos(theta12)*nx+sin(theta12)*ny)*cos(theta63)-(cos(theta12)*ox+sin(theta12)*oy)*sin(theta63))*cos(theta53)-(cos(theta12)*ax+sin(theta12)*ay)*sin(theta53)))-theta26-theta36;
theta47=atan2(real(nz*cos(theta64)-oz*sin(theta64))*cos(theta54)-az*sin(theta54),real(((cos(theta12)*nx+sin(theta12)*ny)*cos(theta64)-(cos(theta12)*ox+sin(theta12)*oy)*sin(theta64))*cos(theta54)-(cos(theta12)*ax+sin(theta12)*ay)*sin(theta54)))-theta27-theta37;
theta48=atan2(real(nz*cos(theta64)-oz*sin(theta64))*cos(theta54)-az*sin(theta54),real(((cos(theta12)*nx+sin(theta12)*ny)*cos(theta64)-(cos(theta12)*ox+sin(theta12)*oy)*sin(theta64))*cos(theta54)-(cos(theta12)*ax+sin(theta12)*ay)*sin(theta54)))-theta28-theta38;
%%
ikine_t=[theta11 theta11 theta11 theta11 theta12 theta12 theta12 theta12 ;
         theta21 theta22 theta23 theta24 theta25 theta26 theta27 theta28;
         theta31 theta32 theta33 theta34 theta35 theta36 theta37 theta38;
         theta41 theta42 theta43 theta44 theta45 theta46 theta47 theta48;
         theta51 theta51 theta52 theta52 theta53 theta53 theta54 theta54;
         theta61 theta61 theta62 theta62 theta63 theta63 theta64 theta64];
ikine_t=ikine_t';
ikine_t = real(ikine_t);
ikine_t(ikine_t<1e-6 & ikine_t>-1e-6)=0;
