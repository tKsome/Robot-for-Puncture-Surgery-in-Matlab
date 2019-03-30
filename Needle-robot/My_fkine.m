function [T06]=My_fkine(theta)
%输入1x6维数组 角度值
global Link
ToDeg = 180/pi;%Trance the value of angle
ToRad = pi/180;
SDH=[theta(1)*ToRad   Link(2).dz   Link(2).dx   Link(2).alf;
     theta(2)*ToRad   Link(3).dz   Link(3).dx   Link(3).alf;
     theta(3)*ToRad   Link(4).dz   Link(4).dx   Link(4).alf;
     theta(4)*ToRad   Link(5).dz   Link(5).dx   Link(5).alf;
     theta(5)*ToRad   Link(6).dz   Link(6).dx   Link(6).alf;
     theta(7)*ToRad   Link(8).dz   Link(8).dx   Link(8).alf];

 T01=[cos(SDH(1,1))  -sin(SDH(1,1))*cos(SDH(1,4))   sin(SDH(1,1))*sin(SDH(1,4))    SDH(1,3)*cos(SDH(1,1));
      sin(SDH(1,1))   cos(SDH(1,1))*cos(SDH(1,4))  -cos(SDH(1,1))*sin(SDH(1,4))    SDH(1,3)*sin(SDH(1,1));
      0               sin(SDH(1,4))                 cos(SDH(1,4))                  SDH(1,2);
      0               0                             0                              1];
 T12=[cos(SDH(2,1))  -sin(SDH(2,1))*cos(SDH(2,4))   sin(SDH(2,1))*sin(SDH(2,4))    SDH(2,3)*cos(SDH(2,1));
      sin(SDH(2,1))   cos(SDH(2,1))*cos(SDH(2,4))  -cos(SDH(2,1))*sin(SDH(2,4))    SDH(2,3)*sin(SDH(2,1));
      0               sin(SDH(2,4))                 cos(SDH(2,4))                  SDH(2,2);
      0               0                             0                              1];
 T23=[cos(SDH(3,1))  -sin(SDH(3,1))*cos(SDH(3,4))   sin(SDH(3,1))*sin(SDH(3,4))    SDH(3,3)*cos(SDH(3,1));
      sin(SDH(3,1))   cos(SDH(3,1))*cos(SDH(3,4))  -cos(SDH(3,1))*sin(SDH(3,4))    SDH(3,3)*sin(SDH(3,1));
      0               sin(SDH(3,4))                 cos(SDH(3,4))                  SDH(3,2);
      0               0                             0                              1];
 T34=[cos(SDH(4,1))  -sin(SDH(4,1))*cos(SDH(4,4))   sin(SDH(4,1))*sin(SDH(4,4))    SDH(4,3)*cos(SDH(4,1));
      sin(SDH(4,1))   cos(SDH(4,1))*cos(SDH(4,4))  -cos(SDH(4,1))*sin(SDH(4,4))    SDH(4,3)*sin(SDH(4,1));
      0               sin(SDH(4,4))                 cos(SDH(4,4))                  SDH(4,2);
      0               0                             0                              1];
 T45=[cos(SDH(5,1))  -sin(SDH(5,1))*cos(SDH(5,4))   sin(SDH(5,1))*sin(SDH(5,4))    SDH(5,3)*cos(SDH(5,1));
      sin(SDH(5,1))   cos(SDH(5,1))*cos(SDH(5,4))  -cos(SDH(5,1))*sin(SDH(5,4))    SDH(5,3)*sin(SDH(5,1));
      0               sin(SDH(5,4))                 cos(SDH(5,4))                  SDH(5,2);
      0               0                             0                              1];
 T56=[cos(SDH(6,1))  -sin(SDH(6,1))*cos(SDH(6,4))   sin(SDH(6,1))*sin(SDH(6,4))    SDH(6,3)*cos(SDH(6,1));
      sin(SDH(6,1))   cos(SDH(6,1))*cos(SDH(6,4))  -cos(SDH(6,1))*sin(SDH(6,4))    SDH(6,3)*sin(SDH(6,1));
      0               sin(SDH(6,4))                 cos(SDH(6,4))                  SDH(6,2);
      0               0                             0                              1];
 T566=[1 0 0 0;
     0 1 0 0;
     0 0 1 theta(6);
     0 0 0 1];

 T06=T01*T12*T23*T34*T45*T566*T56;
 %定零
 T06(T06<1e-6&T06>-1e-6)=0;
