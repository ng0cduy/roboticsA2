
%%
clf
clear all
clc
a = DensoVS060(false,transl(0,0,0),'denso');
% a.model.teach;
hold on;
green= goods('green',transl(-0.4,0,0)*troty(pi));
%%
a.Animate(transl(-0.5,0.2,0)*troty(pi),50,green);
a.Animate(transl(-0.5,-0.4,0)*troty(pi),50,green);


