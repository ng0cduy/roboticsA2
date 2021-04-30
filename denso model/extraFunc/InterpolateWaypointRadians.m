%% InterpolateWaypointRadians
% Given a set of waypoints, finely intepolate them
function qMatrix = InterpolateWaypointRadians(waypointRadians,maxStepRadians)
        if nargin < 2
            maxStepRadians = deg2rad(1);
        end

        qMatrix = [];
        for i = 1: size(waypointRadians,1)-1
            qMatrix = [qMatrix ; FineInterpolation(waypointRadians(i,:),waypointRadians(i+1,:),maxStepRadians)]; %#ok<AGROW>
        end
end
%% FineInterpolation
% Use results from Q2.6 to keep calling jtraj until all step sizes are
% smaller than a given max steps size
function qMatrix = FineInterpolation(q1,q2,maxStepRadians)
        if nargin < 3
            maxStepRadians = deg2rad(1);
        end

        steps = 2;
        while ~isempty(find(maxStepRadians < abs(diff(jtraj(q1,q2,steps))),1))
            steps = steps + 1;
        end
        qMatrix = jtraj(q1,q2,steps);
end