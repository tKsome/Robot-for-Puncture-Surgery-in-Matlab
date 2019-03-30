close all;
clear;
clc;
p=imread('Library2.jpg');
J=imresize(p, [300,300],'nearest');
g=rgb2gray(J); % 转为灰阶图
imshow(g);
pause;
gg=double(g); % 转为数值矩阵
gg=1-gg/255; % 将彩色值转为 0-1 的渐变值
[x,y]=size(gg); % 取原图大小
[X,Y]=meshgrid(1:y,1:x); % 以原图大小构建网格
mesh(X,Y,gg); % 网格上画出图像
colormap gray; 
 