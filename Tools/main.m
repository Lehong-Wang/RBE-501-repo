

clear, clc, close all
addpath('utils');

plotOn = true;
nTests = 20;  % number of random test configurations

IKmethod = "transpose";


%% create manipulator

% robot length values (meters)
L0 = 0.3;
L1 = 0.3;
L2 = 0.3;

robot = SerialLink([Revolute('a', 0, 'd', L0, 'alpha', -pi/2, 'offset', pi/2), ...
                    Revolute('a', L1, 'd', 0, 'alpha', 0), ...
                    Revolute('a', L2, 'd', 0, 'alpha', pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', -pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', -pi/2, 'offset', -pi/2), ...
                    Revolute('a', 0, 'd', 0, 'alpha', 0)], 'name', 'Elbow Manipulator'); 

% Joint limits
qlim = [-pi/2  pi/2;  % q(1)
        -pi/4  pi/2;  % q(2)
        0      pi/3;  % q(3)
        -pi/2  pi/2;  % q(4)
        -pi/2  pi/2;  % q(5)
        -pi/2  pi/2]; % q(6)

% Display the manipulator in the home configuration
q = zeros(1,6);
robot.plot(q);






%% Calculate the screw axes and home configuration
% Let us calculate the screw axis for each joint
% Put all the axes into a 6xn matrix S, where n is the number of joints

S = [ 0 0 1 0 0 0;
     -1 0 0 -cross([-1 0 0], [0 0 L0]);
     -1 0 0 -cross([-1 0 0], [0 L1 L0]);
      0 0 1 -cross([0 0 1], [0 L1+L2 0]);
     -1 0 0 -cross([-1 0 0], [0 L1+L2 L0]);
      0 1 0 -cross([0 1 0], [0 0 L0])]';

R_home = [0 0 1; 1 0 0; 0 1 0]';
t_home = [0 L1+L2 L0]';
M = [R_home t_home; 0 0 0 1];






%% Inverse Kinematics
fprintf('----------------------Inverse Kinematics Test--------------------\n');
fprintf(['Testing ' num2str(nTests) ' random configurations.\n']);
fprintf('Progress: ');
nbytes = fprintf('0%%');



% Calculate the coordinates representing the home pose
currentXYZ = M(1:3,4);

% Set the current joint variables
currentQ = zeros(1,6);

% Generate the target configurations    
t = linspace(-pi, pi, nTests);
X = 0.15 .* t .* sin( pi * .872*sin(t)./t);
Y = 0.4 .* ones(1, nTests);
Z = 0.15 .* -abs(t) .* cos(pi * sin(t)./t) + 0.5;

targetXYZ = [X; Y; Z];

% Display the robot and the points
if plotOn
    robot.plot(currentQ); hold on;
    scatter3(X, Y, Z, 'filled');
end

tic
for ii = 1 : nTests
    fprintf(repmat('\b',1,nbytes));
    nbytes = fprintf('%0.f%%', ceil(ii/nTests*100));
        
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
                   
        robot.maniplty(currentQ);
        
        if plotOn
            try
                robot.plot(currentQ);
                drawnow;
            catch e
                continue;
            end
        end
    end
end

toc

fprintf('\nTest passed successfully.\n');



 
 


