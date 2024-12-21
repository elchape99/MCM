function [q] = KinematicSimulation(q, q_dot, ts, q_min, q_max)
%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration

    % Updating q
    q = q + q_dot * ts;

    numberOf_q = size(q, 2);
    for i = 1:numberOf_q
        if q(i) < q_min(i)
            q(i) = q_min(i);
        end
        if q(i) > q_max(i)
            q(i) = q_max(i);
        end
    end

    % Saturating the joint velocities 


end