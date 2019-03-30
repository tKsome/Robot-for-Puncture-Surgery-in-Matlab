function a=My_traj(thi1,thf1,tb1,tf1,m1,l1,v1,b1,Jm1,Dm1,Ra1,La1,Kt1,Ke1,N1,g1)
global thi thf tb tf m l v b Jm Dm Ra La Kt Ke N g
ToDeg = 180/pi;%Trance the value of angle
ToRad = pi/180;
thi = thi1*ToRad;
thf = thf1*ToRad;
tb = tb1;
tf = tf1;
m = m1;
l = l1;
v = v1;
b =b1;
Jm = Jm1;
Dm = Dm1;
Ra = Ra1;
La = La1;
Kt =Kt1;
Ke =Ke1;
N = N1;
g = g1;
a=( thf-thi )/( tb*( tf-tb ) );
end