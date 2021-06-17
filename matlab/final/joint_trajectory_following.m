%% NOTE
% 1. Open the file named "indy7_scene_matlab_api.ttt"
% 2. Run the simulation of the Coppeliasim file
% 3. Run the matlab file(current file), then you can find out the Indy7
% follow certain trajecotry.
% 4. If you want to put another trajectory, then edit below code
% 5. You can refer to Remote API functions https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm
% 6. If you have any question, feel free to ask the TA via
% E-mail (kdw2917@postech.ac.kr)


%%
clear all
close all
clc

sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
       
    %joints handles
    h = [0,0,0,0,0,0];
    for i=1:6
    [r, h(i)]= sim.simxGetObjectHandle(clientID, convertStringsToChars("Revolute_joint"+string(i)), sim.simx_opmode_blocking);
    end
    
    % n, duration has no big meaning
    n = 100;
    duration = 0.01;

    % You can change this code ------------------------------------------
    joint_pos_mat1 = [linspace(0, pi/2, n); linspace(0, -pi/3, n); linspace(0, pi/4, n);...
                    linspace(0,pi/3,n); linspace(0,pi/2,n); linspace(0,-pi/4,n)];
    joint_pos_mat2 = [linspace(pi/2, 0, n); linspace(-pi/3, 0, n); linspace(pi/4, 0, n);...
                    linspace(pi/3, 0, n); linspace(pi/2, 0, n); linspace(-pi/4, 0, n)];
    

    while true
        for i=1:n
            tstart = tic;
            for j=1:6
                sim.simxSetJointTargetPosition(clientID, h(j), joint_pos_mat1(j, i), sim.simx_opmode_streaming);
            end
            dt = toc(tstart);
            if dt<duration
                pause(duration-dt);
            end
        end
        pause(3);
        
        for i=1:n
            tstart = tic;
            for j=1:6
                sim.simxSetJointTargetPosition(clientID, h(j), joint_pos_mat2(j, i), sim.simx_opmode_streaming);
            end
            dt = toc(tstart);
            if dt<duration
                pause(duration-dt);
            end
        end
        pause(3);
    end
    % You can change this code ------------------------------------------
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!
disp('Program ended');  
