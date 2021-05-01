classdef VisServo<handle
    properties(Constant)
        qz=zeros(1,6);
    end
    properties(Access=public)
        pStar = [662 362 362 662; 362 362 662 662];
%         q0=[0;0;0;0;0;0];
        fps;
        lambda = 0.6;
        ksteps = 0;
        pose;
        
    end
    methods (Access=public)
        function self=VisServo(r,object)
            self.fps=60;
            P=object.P;
            q0=[0;0;0;0;0;0];
            cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
                                'resolution', [1024 1024],...
                                'centre', [512 512],'name', 'DensoCam');
                            
            Tc0= r.model.fkine(q0);
            r.model.animate(q0');
            drawnow();
            cam.plot_camera('Tcam',Tc0, 'label','scale',0.05);
            lighting gouraud
            light;
            hold on;
            depth = mean (P(1,:));
%             cam.clf()
            cam.plot(self.pStar, '*'); % create the camera view
            cam.hold(true);
            cam.plot(P);
            history = [];
            while (self.ksteps <= 200)
                    self.ksteps = self.ksteps + 1;

                    % compute the view of the camera
                    uv = cam.plot(P);

                    % compute image plane error as a column
                    e = self.pStar-uv;   % feature error
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
                        v = self.lambda * pinv(J) * e;
                    catch
%                         status = -1;
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
                    q = q0 + (1/self.fps)*qp;
                    r.model.animate(q');

                    %Get camera location
                    Tc = r.model.fkine(q);
                    cam.T = Tc;
                    self.pose = Tc;

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

                     pause(1/self.fps);

                    %update current joint position
                    q0 = q;
             end %loop finishes
        end
    end
end