

function q = ikin(S, M, currentQ, targetPose)
%     disp("IKin")
    T = fkine(S, M, currentQ);

    currentPose = MatrixLog6(T);
    currentPose = [currentPose(3,2) ...
                   currentPose(1,3) ...
                   currentPose(2,1) ...
                   currentPose(1:3,4)']';


    loop_count = 0;

    while norm(targetPose - currentPose) > 1e-10 && loop_count < 10000
        loop_count = loop_count + 1;
%     disp("Loop here")
        J = jacob0(S, currentQ);

        scale = 0.08;

        % if loop_count < 500
        %         scale = 0.1;
        % % elseif loop_count >= 50000 && loop_count < 100000
        % %         scale = 0.05;
        % % elseif loop_count >= 100000 && loop_count < 1000000
        % %         scale = 0.01;
        % else
        %         scale = 0.005;
        %         % break;
        % end

        deltaQ = scale * pinv(J) * (targetPose - currentPose);
%         DPose = DPose / norm(DPose) * deltaPose
        currentQ = currentQ + deltaQ';
%         error = norm(targetPose - currentPose)
        T = fkine(S, M, currentQ);
        % disp(loop_count + "/t" + norm(targetPose - currentPose) + "/t" + scale);

        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';
        
        % if scale == 0.005
        %         break;
        % end

    end
    


    q = currentQ;
    q = mod(q + pi, 2*pi) - pi
end


