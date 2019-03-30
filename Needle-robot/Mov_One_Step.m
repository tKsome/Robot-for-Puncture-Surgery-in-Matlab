function Mov_One_Step(th1,th2,th3,th4,th5,dz6,th7,fcla)
global Link
% ToDeg = 180/pi;%Trance the value of angle
% ToRad = pi/180;
% th1=th1*ToDeg;
% th2=th2*ToDeg;
% th3=th3*ToDeg;
% th4=th4*ToDeg;
% th5=th5*ToDeg;
% th6=th6*ToDeg;
UR5_Mdl;
radius    = 25;  %25
len       = 60;  %60
joint_col = 0;

plot3(0,0,0,'ro'); 


 Link(2).th= Link(2).th+th1*pi/180;
 Link(3).th=Link(3).th+th2*pi/180;
 Link(4).th=Link(4).th+th3*pi/180;
 Link(5).th=Link(5).th+th4*pi/180;
 Link(6).th=Link(6).th+th5*pi/180;
 Link(7).dz=Link(7).dz+dz6; 
 Link(8).th=Link(8).th+th7*pi/180; 
 %for initial position
 
%   Link(2).th= th1*pi/180;
%  Link(3).th=th2*pi/180;
%  Link(4).th=th3*pi/180;
%  Link(5).th=th4*pi/180;
%  Link(6).th=th5*pi/180;
%  Link(7).th=th6*pi/180;

p0=[0,0,0]';


for i=1:8
Matrix_DH(i);
end


for i=2:8

      Link(i).A=Link(i-1).A*Link(i).A;
      Link(i).p= Link(i).A(:,4);
      Link(i).n= Link(i).A(:,1);
      Link(i).o= Link(i).A(:,2);
      Link(i).a= Link(i).A(:,3);
      Link(i).R=[Link(i).n(1:3),Link(i).o(1:3),Link(i).a(1:3)];
      Connect3D(Link(i-1).p,Link(i).p,'b',2); hold on;
      plot3(Link(i).p(1),Link(i).p(2),Link(i).p(3),'rx');hold on;
      if i<=8
          DrawCylinder(Link(i-1).p, Link(i-1).R * Link(i).az, radius,len, joint_col); hold on;
      end 
end
% view(125,52);
% set (gcf,'Position',[650,100,700,600])
axis([-1000,1000,-1000,1000,-1000,1100]);
xlabel('x');
ylabel('y'); 
zlabel('z');
grid on;
drawnow;
if(fcla)
    cla;
end





