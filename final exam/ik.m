% RBE 501 - Robot Dynamics - Spring 2023
% Midterm Exam
% Worcester Polytechnic Institute
%
% Instructor: L. Fichera <lfichera@wpi.edu>
% Last modified: 02/21/2023
clear, clc, close all
addpath('utils');

%% First, execute poe.m to load the S and M matrices
poe
close all

%% Generate and display the path that the robot has to trace
nPts = 10;
path = [0.2 * ones(1,nPts);
        linspace(-0.45, 0.45, 10);
        -0.15 * ones(1,nPts)];

path(3,2:end-1) = path(3,2:end-1) + 0.10;

robot.plot(zeros(1,n)), hold on;
scatter3(path(1,:), path(2,:), path(3,:), 'filled');
drawnow;





IKmethod = "pseudoinverse";
plotOn = true;


targetXYZ = path;
currentXYZ = [0.55 0 0]';

currentQ = zeros(1,4);

qList = [];


tic
for ii = 1 : width(targetXYZ)
%     fprintf(repmat('\b',1,nbytes));
%     nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
        
    while norm(targetXYZ(:,ii) - currentXYZ) > 1e-3
        Ja = jacoba(S,M,currentQ);  

        if strcmp(IKmethod, 'pseudoinverse')
            deltaQ = pinv(Ja) * (targetXYZ(:,ii) - currentXYZ);
            
        elseif strcmp(IKmethod, 'transpose')
            error_term = (targetXYZ(:,ii) - currentXYZ);
            alpha = dot(error_term, Ja*Ja'*error_term) / ...
                    dot(Ja*Ja'*error_term, Ja*Ja'*error_term);
                     
            deltaQ = alpha * Ja' * (targetXYZ(:,ii) - currentXYZ);
            
        elseif strcmp(IKmethod, 'DLS')
            lambda = 0.1;
            deltaQ = Ja' * pinv(Ja*Ja' + lambda^2 * eye(3)) * (targetXYZ(:,ii) - currentXYZ);
            
        else 
            error('IK Method not defined.');
        end
        
        currentQ = currentQ + deltaQ';
        
        T = fkine(S,M,currentQ);
        currentXYZ = T(1:3,4);
             
        %robot.maniplty(currentQ);
        
        if plotOn
            try
                robot.plot(currentQ);
                drawnow;
            catch e
                continue;
            end
        end
    end
    qList = [qList; currentQ]
end

qList
toc

fprintf('\nTest passed successfully.\n');













%% Solve the inverse kinematics
% Set the current joint variables

% Create a vector where we are going to store all the solutions for the IK
% qList = zeros(size(path,2), n);

% *** YOUR CODE HERE ***
% error('The Inverse Kinematics has not been implemented yet.') % *** COMMENT THIS OUT ***

%% Carry out the pick-and-place task ten times
close all
robot.plot(repmat([qList; flip(qList)], 10, 1), ...
          'trail', {'r', 'LineWidth', 5});