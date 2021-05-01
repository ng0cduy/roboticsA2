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
        history = [];
        vel_p = [];
        uv_p = [];
        cam_ = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
                                'resolution', [1024 1024],...
                                'centre', [512 512],'name', 'DensoCam');
        
    end
    methods (Access=public)
        function self=VisServo(r,object)
            cam = self.cam_;
            self.fps=60;
            P=object.P;
            q0=r.model.getpos()';
            r.Reset;
                            
            Tc0= r.model.fkine(q0);
%             Move the robot to the righ position
            r.Animate('jtraj',Tc0,30);
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

                    self.history = [self.history hist];

                     pause(1/self.fps);

                    %update current joint position
                    q0 = q;
             end %loop finishes
        end
        
        function plot_graph(self)
            figure()            
            plot_p(self.history,self.pStar,self.cam_)
            figure()
            plot_camera(self.history)
            figure()
            plot_vel(self.history)
            figure()
            plot_robjointpos(self.history)
            figure()
            plot_robjointvel(self.history)

        end
    end
end

%% Functions for plotting

 function plot_p(history,uv_star,camera)
            %VisualServo.plot_p Plot feature trajectory
            %
            % VS.plot_p() plots the feature values versus time.
            %
            % See also VS.plot_vel, VS.plot_error, VS.plot_camera,
            % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.
            
            if isempty(history)
                return
            end
            figure();
            clf
            hold on
            % image plane trajectory
            uv = [history.uv]';
            % result is a vector with row per time step, each row is u1, v1, u2, v2 ...
            for i=1:numcols(uv)/2
                p = uv(:,i*2-1:i*2);    % get data for i'th point
                plot(p(:,1), p(:,2))
            end
            plot_poly( reshape(uv(1,:), 2, []), 'o--');
            uv(end,:)
            if ~isempty(uv_star)
                plot_poly(uv_star, '*:')
            else
                plot_poly( reshape(uv(end,:), 2, []), 'rd--');
            end
            axis([0 camera.npix(1) 0 camera.npix(2)]);
            set(gca, 'Ydir' , 'reverse');
            grid
            xlabel('u (pixels)');
            ylabel('v (pixels)');
            hold off
        end

       function plot_vel(history)
            %VisualServo.plot_vel Plot camera trajectory
            %
            % VS.plot_vel() plots the camera velocity versus time.
            %
            % See also VS.plot_p, VS.plot_error, VS.plot_camera,
            % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.
            if isempty(history)
                return
            end
            clf
            vel = [history.vel]';
            plot(vel(:,1:3), '-')
            hold on
            plot(vel(:,4:6), '--')
            hold off
            ylabel('Cartesian velocity')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('v_x', 'v_y', 'v_z', '\omega_x', '\omega_y', '\omega_z')
        end

        function plot_camera(history)
            %VisualServo.plot_camera Plot camera trajectory
            %
            % VS.plot_camera() plots the camera pose versus time.
            %
            % See also VS.plot_p, VS.plot_vel, VS.plot_error,
            % VS.plot_jcond, VS.plot_z, VS.plot_error, VS.plot_all.

            if isempty(history)
                return
            end
            clf
            % Cartesian camera position vs time
            T = reshape([history.Tcam], 4, 4, []);
            subplot(211)
            plot(transl(T));
            ylabel('camera position')
            grid
            subplot(212)
            plot(tr2rpy(T))
            ylabel('camera orientation')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('R', 'P', 'Y');
            subplot(211)
            legend('X', 'Y', 'Z');
        end

        function plot_robjointvel(history)
          
            if isempty(history)
                return
            end
            clf
            vel = [history.qp]';
            plot(vel(:,1:6), '-')
            hold on
            ylabel('Joint velocity')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('\omega_1', '\omega_2', '\omega_3', '\omega_4', '\omega_5', '\omega_6')
        end

 function plot_robjointpos(history)
          
            if isempty(history)
                return
            end
            clf
            pos = [history.q]';
            plot(pos(:,1:6), '-')
            hold on
            ylabel('Joint angle')
            grid
            xlabel('Time')
            xaxis(length(history));
            legend('\theta_1', '\theta_2', '\theta_3', '\theta_4', '\theta_5', '\theta_6')
        end
     