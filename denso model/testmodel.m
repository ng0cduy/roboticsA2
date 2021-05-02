
%%
clc;
% clear all;
close all;
a = DensoVS060(false,transl(0,0,0),'denso');
hold on;
brick= goods('red.ply',transl(0.3,0,0.1)*troty(pi));
% lt =LightCurtain(true);

% a.model.teach;
% hold on;
% blue= goods('blue.ply',transl(0.4,0,0.05)*troty(pi));
% red= goods('red.ply',transl(-0.4,0,0.05)*troty(pi));
% table=goods('table1.ply',transl(0,0,-0.3));
% % %% Test rmrc
% % %%
% a.Reset;
% a.Animate('jtraj',transl(0.4,0.2,0.02)*troty(pi),50,table,blue);
% keyboard;
% a.Animate('rmrc',transl(0.4,-0.2,0.05)*troty(pi),50);
% 
% a.Reset;
% camview = EEcam(a);
% pause(1);
% view(0,90)
% keyboard;
%% camera testing
b=VisServo(a,brick);


