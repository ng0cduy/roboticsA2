
%%
clf
clear all
a = DensoVS060(false,transl(0,0,0),'denso');
% a.model.teach;
hold on;
green= goods('green',transl(0.4,0.3,-0.13)*troty(pi));

%%
keyboard
clc
pose = transl(0,0,0.5)*troty(pi);
q = a.IKine(pose);
a.Animate(green.pos_,50);
% a.Reset()

