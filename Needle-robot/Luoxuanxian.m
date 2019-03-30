function [ice_frame,len]=Luoxuanxian(x_label,y_label,z_label)
%%%%%%%%%ice
t = 0:0.1:10*pi;
A = 1;
w = 1;
sita = 0;
for ii = 1:length(t)
    x(ii) = A *cos(w*t(ii) + sita);
    y(ii) = A *sin(w*t(ii) + sita);
    A = A+ 0.1;
end
z = t*8;
z = fliplr(z);
z = z+z_label;
x = x*5;
x = x+x_label;
y = y*5;
y = y+y_label;
ice_frame=[x;y;z];
ice_frame=ice_frame';
len=length(ice_frame(:,1));
r=ice_frame(len,1)-ice_frame(1,1);
%%%%%%%%%%cycle
t = -pi:0.1:pi;
x_cycle=r*sin(t-pi/2)+x_label;
y_cycle=r*cos(t-pi/2)+y_label;
z_cycle=0*t+z_label;
ice_frame=ice_frame';
ice_frame=[ice_frame(1,:) x_cycle;ice_frame(2,:) y_cycle;ice_frame(3,:) z_cycle];
ice_frame=ice_frame';
len=length(ice_frame(:,1));
%%%%%%%%%%cone
t = 0:0.1:10*pi;
A = 1;
w = 5;
sita =0;
for ii = 1:length(t)
    x(ii) = A *cos(w*t(ii) + sita);
    y(ii) = A *sin(w*t(ii) + sita);
    A = A+ 0.1;
end
z = -t*20;
% z = fliplr(z);
z = z+z_label;
x = x*5;
x = x+x_label;
x = fliplr(x);
y = y*5;
y = y+y_label;
y = fliplr(y);
% lent=length(t);
ice_frame=ice_frame';
ice_frame=[ice_frame(1,:) x;ice_frame(2,:) y;ice_frame(3,:) z];
ice_frame=ice_frame';
len=length(ice_frame(:,1));
end