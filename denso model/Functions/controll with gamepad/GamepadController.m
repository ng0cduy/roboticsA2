function GamepadController(denso,gui)
    denso.Reset();
    id = 1;
    joy = vrjoystick(id);
    caps(joy)
    q = denso.model.getpos();
    duration = 100;
    dt = 0.15;
    n =0;
    denso.model.delay=0.001;
    kLinear = 0.15;
    kAngular = 0.7;
    lambda =0.01;
    %%
    tic;
    while(toc <duration)
        n = n+1;
        [axes, buttons, povs] = read(joy);
        
        J = denso.model.jacob0(q);
        JDLS_inv = (J'*J + lambda*eye(6))\(J');
        vx = kLinear*axes(1);
        vy = kLinear*axes(2);
        vz = kLinear*axes(3);
        wx = kAngular*(buttons(2)-buttons(1));
        wy = kAngular*(buttons(7)-buttons(5));
        wz = kAngular*(buttons(8)-buttons(6));
        xdot = [vx;vy;vz;wx;wy;wz];
        qdot = JDLS_inv*xdot;
%         qdot = J\xdot;
        q = q + dt*qdot';
        % pass value to gui
        gui.q1EditField.Value = num2str(rad2deg(q(1,1)));
        gui.q2EditField.Value = num2str(rad2deg(q(1,2)));
        gui.q3EditField.Value = num2str(rad2deg(q(1,3)));
        gui.q4EditField.Value = num2str(rad2deg(q(1,4)));
        gui.q5EditField.Value = num2str(rad2deg(q(1,5)));
        gui.q6EditField.Value = num2str(rad2deg(q(1,6)));
        % conver to xyz roll pitch yaw
        pose = denso.FKine(q);
        RPY = tr2rpy(pose,'deg');
        gui.xEditField.Value = num2str(round(pose(1,4),4));
        gui.yEditField.Value = num2str(round(pose(2,4),4));
        gui.zEditField.Value = num2str(round(pose(3,4),4));
        gui.rEditField.Value = num2str(round(RPY(1),4));
        gui.pEditField.Value = num2str(round(RPY(2),4));
        gui.y_EditField.Value = num2str(round(RPY(3),4));
        denso.model.animate(q); 
        for joint = 1:1:6
            if q(joint) < denso.model.qlim(joint,1)
                q(joint) = denso.model.qlim(joint,1);
                qdot(joint) =0;
            elseif q(joint) > denso.model.qlim(joint,2)
                q(joint) = denso.model.qlim(joint,2);
                qdot(joint) = 0;
            end 
        end
        % wait until loop time elapsed
        if (toc > dt*n)
            warning('Loop %i took too much time - consider increating dt',n);
        end
        if (buttons(3) ==1)
            denso.Reset();
            q = denso.qz;
        end
        while (toc < dt*n) % wait until loop time (dt) has elapsed 
        end
        
        if (buttons(4) ==1)
            denso.Reset();
            break;
        end 
    end
end 