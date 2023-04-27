

function traj = make_trajectory(type, params)

% Input
% type - a string indicating the type of trajectory that you want to generate.
%                Acceptable values: {'cubic' | 'quintic'}
%
% params - a structure containing the prescribed trajectory parameters
%   params.t  - 2-vector prescribing the initial and final time
%   params.dt - desired time step
%   params.q - 2-vector prescribing the starting and final positions
%   params.v - 2-vector describing the starting and final velocities
%   params.a - 2-vector describing the starting and final accelerations (only for quintic polynomials)
%
% Return
% traj - a structure containing the trajectory
%   traj.t - n-vector representing time
%   traj.q - n-vector representing position over time
%   traj.v - n-vector representing velocity over time
%   traj.a - n-vector representing acceleration over time

    q0 = params.q(1);
    qf = params.q(2);
    v0 = params.v(1);
    vf = params.v(2);
    t0 = params.t(1);
    tf = params.t(2);
    dt = params.dt;

    if strcmp(type, 'cubic')
        A = [0      0       t0^3    t0^2    t0      1;
             0      0       3*t0^2  2*t0    1       0;
             0      0       0       0       0       0;
             0      0       tf^3    tf^2    tf      1;
             0      0       3*tf^2  2*tf    1       0;
             0      0       0       0       0       0;];
        
        a0 = 0;
        af = 0;
        
    elseif strcmp(type, 'quintic')
        A = [t0^5   t0^4    t0^3    t0^2    t0      1;
             5*t0^4  4*t0^3  3*t0^2  2*t0   1       0;
             20*t0^3 12*t0^2 6*t0    2      0       0;
             tf^5   tf^4    tf^3    tf^2    tf      1;
             5*tf^4  4*tf^3  3*tf^2  2*tf   1       0;
             20*tf^3 12*tf^2 6*tf    2      0       0;];

        a0 = params.a(1);
        af = params.a(2);

    else
        disp("type must be either 'cubic' or 'quintic'")
        traj = 0;
        return
    end


    B = [q0 v0 a0 qf vf af]';
    % solve A * X = B
    X = pinv(A) * B;


    t = transpose(t0:dt:tf);

    % put t into coefficient matrix
    q = [t.^5   t.^4    t.^3    t.^2    t       t.^0] * X;
    v = [5*t.^4  4*t.^3  3*t.^2  2*t    t.^0    t*0] * X;
    a = [20*t.^3 12*t.^2 6*t    2*t.^0  0*t     0*t] * X;

    traj.t = t;
    traj.q = q;
    traj.v = v;
    traj.a = a;


end

