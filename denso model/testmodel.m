
%%
clc;
clear all;
close all;
a = DensoVS060(false,transl(0.8,0.32,0)*trotz(-pi/2),'denso_1');
hold on;
brick= goods('red.ply',transl(0.8,-0.1,0.2));
%% camera testing
% b=VisServo(a,brick);


