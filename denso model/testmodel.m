% close all;
clf
clc
clear all
%%
clf
clear all
a = DensoVS060(false,transl(0,0,0),'denso');
% a.model.teach;
hold on;
green= goods('green',transl(0.4,0,-0.13));

%%
clc
pose = transl(0,0,0.5)*trotz(45,'deg');
q = a.IKine(pose);
a.Animate(pose,50);
a.Reset()

