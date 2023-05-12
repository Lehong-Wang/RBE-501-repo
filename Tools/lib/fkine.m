

function T = fkine(S,M,q,frame)

    % Forward Kinamatics
    % Implemented with PoE
    % Input:
    %   S : 6xn of Skew Axis for each joint
    %   M : 4x4 Home Configuration
    %   q : 6x1 vector of joint values
    %   frame : {'space', 'body'}
    % Output:
    %   T : transformation between base frame and EE frame

    % default frame='space'
    if nargin == 3
        frame = 'space';
    end
    % frame
    T = eye(4);
    for i = 1: width(S)
        Si = S(:,i);
        qi = q(i);
        TF = twist2ht(Si, qi);
        T = T * TF;
    end

    if strcmp(frame, 'space')
    % if frame == 'space'
        T = T * M;
    elseif strcmp(frame, 'body')
    % if frame == 'body'
        T = M * T;
    else
        disp("Not Valid Frame Name")
    end
end