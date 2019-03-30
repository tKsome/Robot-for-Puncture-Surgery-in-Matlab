clear;
close;
ToRad=pi/180;
ToDeg=180/pi;
thi=30*ToRad;
thf=100*ToRad;
tb=1;
tf=6;

a=( thf-thi )/( tb*( tf-tb ) );

m=1;   %ÔØºÉÖÊÁ¿kg
l=1;   %±Û³¤1m 
v=10;  %ÔØºÉ×èÄá
b=0;   %¿âÂ×Ä¦²Á

Jm=1e-5;
Dm=1e-6;
Ra=10;
La=4.4e-3;
Kt=0.05; %Nm/A
Ke=Kt;    %Vs/rad
N=5;
g=0;

sim('Robot_Joint_Prototype',tf);
for n=1:1:length(tout)
    h=plot3([0,l*cos(simout(n))],[0,l*sin(simout(n))],[0,0])
    set(h,'LineWidth',3);
    axis([-1.5,1.5,-1.5,1.5,-1.5,1.5]);
    xlabel('x');
    ylabel('y');
    zlabel('z'); grid on; view(0,90);
    str=sprintf('Time=%2.3fs',tout(n));
    text( 0.8,1.3, 0,str,'FontSize',12 );
    str=sprintf('Theta=%2.3f¡ã',simout(n)*ToDeg);
    text( 0.8,1.1, 0,str,'FontSize',12 );
    drawnow;
%     pause(tout(n));
end
    

% thb=thi+0.5*a*tb^2