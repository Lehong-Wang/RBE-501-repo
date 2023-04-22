

function T = fkine(S,M,q,frame)
    % arguments frame='space'; end
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