function [body_frame,len]=body(x_label,y_label,z_label)
t=linspace(0,2*pi,200);
r=100;
rr=100;
xx=r*cos(t);
yy=r*sin(t);
zz=ones(1,200);
x=r*cos(t);
y=r*sin(t);
z=ones(1,200);
for i=0:0.01:1
    x=[x ,xx];
    y=[y, yy];
    z=[z,zz*i];
end
z=rr*z;
% plot3(z,y,abs(x));
z=z+x_label-rr;
y=y+y_label;
x=abs(x)+z_label-0.7893;
body_frame=[z;y;x];
body_frame=body_frame';
len=length(body_frame(:,1));
end


