close all;
clear;
clc;
p=imread('Library2.jpg');
J=imresize(p, [300,300],'nearest');
g=rgb2gray(J); % תΪ�ҽ�ͼ
imshow(g);
pause;
gg=double(g); % תΪ��ֵ����
gg=1-gg/255; % ����ɫֵתΪ 0-1 �Ľ���ֵ
[x,y]=size(gg); % ȡԭͼ��С
[X,Y]=meshgrid(1:y,1:x); % ��ԭͼ��С��������
mesh(X,Y,gg); % �����ϻ���ͼ��
colormap gray; 
 