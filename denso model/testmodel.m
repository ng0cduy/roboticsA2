
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
%% 1.1 Definitions
hold on
% img = imread('red.png');
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
p = cam.plot(P, 'Tcam', Tc0);

%camera view and plotting
cam.clf()
% cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(red.vUpdate', 'Tcam', Tc0); % create the camera view

% this is the 'external' view of the points and the camera
% plot_sphere(red.vUpdate', 0.05, 'r')
grid on
cam.hold(true);

vel_p = [];
uv_p = [];
history = [];

ksteps = 0;
 while true
        ksteps = ksteps + 1;
        
        % compute the view of the camera
        uv = cam.plot(P);
        
        % compute image plane error as a column
        e = pStar-uv;   % feature error
        e = e(:);
        Zest = [];
        
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), P);
            J = cam.visjac_p(uv, pt(3,:) );
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            J = cam.visjac_p(uv, depth );
        end

        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = r.model.jacobn(q0);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end

        %Update joints 
        q = q0 + (1/fps)*qp;
        r.model.animate(q');

        %Get camera location
        Tc = r.model.fkine(q);
        cam.T = Tc;

        drawnow
        
        % update the history variables
        hist.uv = uv(:);
        vel = v;
        hist.vel = vel;
        hist.e = e;
        hist.en = norm(e);
        hist.jcond = cond(J);
        hist.Tcam = Tc;
        hist.vel_p = vel;
        hist.uv_p = uv;
        hist.qp = qp;
        hist.q = q;

        history = [history hist];

         pause(1/fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        q0 = q;
 end %loop finishes

% camview = EEcam(r);
keyboard;

