
%%
clf;
clc
clear all;
a = DensoVS060(false,transl(0,0,0),'denso');
% % a.model.teach;
hold on;
blue= goods('blue.ply',transl(0.4,0,0.05)*troty(pi));
% red= goods('red.ply',transl(-0.4,0,0.05)*troty(pi));
% table=goods('table1.ply',transl(0,0,-0.3));
% %% Test rmrc
% clc
% qMatrix = a.GenerateRMRC(transl(-0.4,0,0)*troty(pi),50);
% %%
% % for i =1:size(qMatrix,1)
% %     a.model.animate(qMatrix(i,:));
% %     drawnow();
% %     pause(0.05);
% % end
% %%
% a.Reset;
% a.Animate('jtraj',transl(-0.4,0.2,0.02)*troty(pi),50,red,table);
% a.Animate('rmrc',transl(-0.4,-0.2,0.05)*troty(pi),50,red,table);
%% camera testing
%% 1.1 Definitions
hold on
img = imread('red.png');
% Create image target (points in the image plane) 
pStar = [662 362 362 662; 362 362 662 662];

%Create 3D points
P=[1.8,1.8,1.8,1.8;
-0.25,0.25,0.25,-0.25;
 1.25,1.25,0.75,0.75];


% Make a UR10
r = DensoVS060(false,transl(0,0,0),'denso');       

%Initial pose
q0 = [0; 0; 0; 0; 0; 0];

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'DensoCamera');

% frame rate
fps = 25;

%Define values
%gain of the controler
lambda = 0.6;
%depth of the IBVS
depth = mean (P(1,:));
%% 1.2 Initialise Simulation (Display in 3D)

%Display UR10
Tc0= r.FKine(q0');
% Tc0= r.model.fkine(r.model.getpos());
r.model.animate(q0');
drawnow

% plot camera and points
cam.T = Tc0;

% Display points in 3D and the camera
% cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);
plot_sphere(P, 0.1, 'b')
lighting gouraud
light

%% 1.3 Initialise Simulation (Display in Image view)
hold on
red= goods('red.ply',transl(0.4,0,0.05)*troty(pi));

% Tc0 = transl(0,0,1)*troty(0.1);
cam.T = Tc0;

%camera view and plotting
cam.clf()
% cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
cam.plot(red.vUpdate', 'Tcam', Tc0); % create the camera view

% this is the 'external' view of the points and the camera
% plot_sphere(red.vUpdate', 0.05, 'r')
% cam.plot_camera(P, 'label','scale',0.3);
% cam.plot_camera(red.vUpdate', 'label','scale',0.04);
grid on


camview = EEcam(r);
keyboard;

