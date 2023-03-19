

function q = ikin(S, M, currentQ, targetPose)

	T = fkine(S, M, currentQ);
    currentPose = MatrixLog6(T);
    currentPose = [currentPose(3,2) ...
                   currentPose(1,3) ...
                   currentPose(2,1) ...
                   currentPose(1:3,4)']';


	loop_count = 0;

    while norm(targetPose - currentPose) > 1e-10 && loop_count < 10000
        loop_count = loop_count + 1;

        J = jacob0(S, currentQ);

        scale = 0.08;

        deltaQ = scale * pinv(J) * (targetPose - currentPose);

        currentQ = currentQ + deltaQ';


        if mod(loop_count, 500) == 0
			disp(loop_count + "   " + norm(targetPose - currentPose) + "   " + scale);
        end

        T = fkine(S, M, currentQ);
        currentPose = MatrixLog6(T);
        currentPose = [currentPose(3,2) ...
                       currentPose(1,3) ...
                       currentPose(2,1) ...
                       currentPose(1:3,4)']';

    end
    
    q = currentQ;
    % convert to -pi ~ pi
    q = mod(q + pi, 2*pi) - pi;
end


