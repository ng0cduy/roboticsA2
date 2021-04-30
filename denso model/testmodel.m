
%%
% clf;
% clc
% clear all;
% a = DensoVS060(false,transl(0,0,0),'denso');
% % % a.model.teach;
% hold on;
% blue= goods('blue.ply',transl(0.4,0,0.05)*troty(pi));
% red= goods('red.ply',transl(-0.4,0,0.05)*troty(pi));
% table=goods('table1.ply',transl(0,0,-0.3));
% % %% Test rmrc
% % %%
% a.Reset;
% a.Animate('jtraj',transl(0.4,0.2,0.02)*troty(pi),50,table,blue);
% a.Animate('rmrc',transl(0.4,-0.2,0.05)*troty(pi),50,table,blue);
% 
% a.Reset;
% camview = EEcam(a);
% keyboard;
%% camera testing


% camview = EEcam(r);
keyboard;

