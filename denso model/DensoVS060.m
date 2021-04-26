classdef DensoVS060<handle
    properties(Access =public)
        base;
        endEffector;
        model;
        qMatrix;
        useGripper = false;   
        name;
        initialPose=zeros(1,7);
        pose
        workspace = [-1 1 -1 1 -0.3 1]; 
    end
    methods (Access = public) %% Class for DensoVS060 robot simulation
        %% Define robot Function  
        function self = DensoVS060(useGripper, base, name)
            self.useGripper = useGripper;  
            self.base = base;
            self.name = name;
            
            self.getDensoVS060();
            self.SetBase(base);
        end
        %% Create Denso Links
        function getDensoVS060(self)       
            L1 = Link([pi     0       0       pi/2    1.001]);
            L2=Link('alpha',pi/2,'a',0, 'd',0.145, 'offset',pi,'qlim',[deg2rad(-360), deg2rad(360)] );
            L3=Link('alpha',0,'a',0.308, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-120), deg2rad(120)]);
            L4=Link('alpha',-pi/2,'a',0, 'd',0, 'offset',0,'qlim',[deg2rad(-160), deg2rad(160)]);
            L5=Link('alpha',pi/2,'a',0, 'd',0.31, 'offset',0,'qlim',[deg2rad(-180), deg2rad(180)]);
            L6=Link('alpha',-pi/2,'a',0, 'd',0, 'offset',pi/2,'qlim',[deg2rad(-360), deg2rad(360)]);
            L7=Link('alpha',0,'a',0, 'd',0.06, 'offset',0,'qlim',[deg2rad(360), deg2rad(360)]);
            L1.qlim = [-0.8 0];            
            self.model = SerialLink([L1 L2 L3 L4 L5 L6 L7],'name',self.name);
%             self.model.teach();
        end
        %% Set Denso Base Location 
        function SetBase(self,base)
            self.model.base=base*trotz(-pi/2)*trotx(pi/2);
            self.PlotandColorUR3();
        end
        
        %% Import PLY files to get the 3D model
        function PlotandColorUR3(self)
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['L',num2str(linkIndex),'.ply'],'tri'); 
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
        end
        %% ForWard Kinematic for UR3
        function FKine(self,q)
              self.endEff = self.model.fkine(q);
        end
        %% InverseKinematic for UR3
        function IKine(self,transform)
    %           self.pose=transform;
              self.pose=self.model.ikcon(transform);
        end
        %% Trajectory using Quintic Polynomial 
        function Animate(self,pose)
              self.IKine(pose);
              self.qMatrix = jtraj(self.model.getpos, self.pose, 15);
%               self.initialPose = self.model.getpos;
              for i = 1:1:15
                   self.model.animate(self.qMatrix(i,:));
                   drawnow();
              end
        end
    end
end