sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
       
    % gripper handles
    gripperName="BarrettHand";
    [~, gripperHandle] = sim.simxGetObjectHandle(clientID, convertStringsToChars(gripperName), sim.simx_opmode_blocking);
    
    % wheel handles
    wheel_joint1="wheel_joint1";
    [~, wheel1Handle] = sim.simxGetObjectHandle(clientID, convertStringsToChars(wheel_joint1), sim.simx_opmode_blocking);
    
    wheel_joint2="wheel_joint2";
    [~, wheel2Handle] = sim.simxGetObjectHandle(clientID, convertStringsToChars(wheel_joint2), sim.simx_opmode_blocking);
    
    wheelHandle = [wheel1Handle wheel2Handle];
    
    % default dt
    duration = 0.05;
    % # of times steps
    n = 10;
    
    % n * duration  = time of simulation
    
    % You can change this code ------------------------------------------
    wheel_ang_vel1 = [linspace(0, -1, n); linspace(0, -1, n);];
    wheel_ang_vel2 = [linspace(0, 1, n); linspace(0, 1, n);];

    while true
        for i=1:n
            tstart = tic;
            
            % close the gripper
            sim.simxSetIntegerSignal(clientID, convertStringsToChars(gripperName+"_close"), 1, sim.simx_opmode_blocking);
            
            for j=1:2
                sim.simxSetJointTargetVelocity(clientID, wheelHandle(j), wheel_ang_vel1(j,i), sim.simx_opmode_streaming);
            end
            dt = toc(tstart);
            if dt<duration
                pause(duration-dt);
            end
        end
        pause(3);
        
        for i=1:n
            tstart = tic;
            
            % close the gripper
            sim.simxSetIntegerSignal(clientID, convertStringsToChars(gripperName+"_close"), 0, sim.simx_opmode_blocking);
            
            for j=1:2
                sim.simxSetJointTargetVelocity(clientID, wheelHandle(j), wheel_ang_vel2(j,i), sim.simx_opmode_streaming);
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