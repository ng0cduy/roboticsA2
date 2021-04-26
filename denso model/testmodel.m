% close all;
clf
clear all
%%
a = DensoVS060(false,transl(0,0,0),'denso');
a.model.teach;

% keyboard;
% temp= [0.6,0.5,0.1];
% a.Animate(transl(temp));
%%
clc
pose = transl(0,0,0.5)*trotz(45,'deg');
q = a.IKine(pose);
a.Animate(pose,50);
% a.model.animate(q);
a.Reset()
% a.Animate(pose,15);
% q = a.model.getpos();
% b = cell(1,15)
% b{1} = transl(1,1,1)
