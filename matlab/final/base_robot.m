sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

Trans = @(x, y, z) [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1];

if (clientID>-1)
    disp('Connected to remote API server');
    
    %joints handles
    h = [0,0,0,0,0,0];
    for i=1:6
    [r, h(i)]= sim.simxGetObjectHandle(clientID, convertStringsToChars("Revolute_joint"+string(i)), sim.simx_opmode_blocking);
    end
    
    % gripper handles
    gripperName="BarrettHand";
    [~, gripperHandle] = sim.simxGetObjectHandle(clientID, convertStringsToChars(gripperName), sim.simx_opmode_blocking);
    
    % wheel handles
    wheel_joint1="wheel_joint1";
    [~, wheel1Handle] = sim.simxGetObjectHandle(clientID, convertStringsToChars(wheel_joint1), sim.simx_opmode_blocking);
    
    wheel_joint2="wheel_joint2";
    [~, wheel2Handle] = sim.simxGetObjectHandle(clientID, convertStringsToChars(wheel_joint2), sim.simx_opmode_blocking);
    
    wheelHandle = [wheel1Handle wheel2Handle];
    
    % open the gripper
    sim.simxSetIntegerSignal(clientID, convertStringsToChars(gripperName+"_close"), 0, sim.simx_opmode_blocking);
    
    % default dt
    duration = 0.01;
    % # of times steps
    n = 100;
    
    % n * duration  = time of simulation
    
    
    %% traj1
    thetastart = [0; 0; 0; 0; 0; 0];

    thetaend = [0; -40*pi/180; -50*pi/180; 0; -90*pi/180; 0]; % [0; -40*pi/180; -80*pi/180; 0; -65*pi/180; 0];
    Tf = 3;
    N = 100;
    method = 3;

    traj1 = JointTrajectory(thetastart, thetaend, Tf, N, method);
    
    for i=1:n
        tstart = tic;
        for j=1:6
            sim.simxSetJointTargetPosition(clientID, h(j), traj1(i, j), sim.simx_opmode_streaming);
        end
        dt = toc(tstart);
        if dt<duration
            pause(duration-dt);
        end
    end
    pause(2);
    
    %% traj 2
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Cartesian Trajectory 2
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% (하강하는 부분)
    % 각각 로드의 길이 (m 단위)
    L1 = 0.4495;
    L2 = 0.2655;
    L3 = 0.228;
    H1 = 0.223;
    H2 = 0.0845;
    W1 = 0.1093;
    W2 = 0.0311;
    W3 = 0.0742;
    W4 = 0.1143;
    W5 = 0.0687;

    q_s1 = [0; 0; 0];
    q_s2 = [-W1; 0; H1];
    q_s3 = [-W1+W2; 0; H1+L1];
    q_s4 = [-W1+W2+W3; 0; H1+L1+L2];
    q_s5 = [-W1+W2+W3-W4; 0; H1+L1+L2+H2];
    q_s6 = [-W1+W2+W3-W4-W5; 0; H1+L1+L2+H2+L3];

    w_s1 = [0;0;1];
    w_s2 = [-1;0;0];
    w_s3 = [-1;0;0];
    w_s4 = [0;0;1];
    w_s5 = [-1;0;0];
    w_s6 = [0;0;1];

    v_s1 = cross(q_s1,w_s1);
    v_s2 = cross(q_s2,w_s2);
    v_s3 = cross(q_s3,w_s3);
    v_s4 = cross(q_s4,w_s4);
    v_s5 = cross(q_s5,w_s5);
    v_s6 = cross(q_s6,w_s6);

    S1 = vertcat(w_s1,v_s1);
    S2 = vertcat(w_s2,v_s2);
    S3 = vertcat(w_s3,v_s3);
    S4 = vertcat(w_s4,v_s4);
    S5 = vertcat(w_s5,v_s5);
    S6 = vertcat(w_s6,v_s6);

    Slist = [S1,S2,S3,S4,S5,S6];

    M = [1 0 0 -W1+W2+W3-W4-W5;
         0 1 0 0;
         0 0 1 H1+L1+L2+H2+L3;
         0 0 0 1];

    thetastart1 = thetaend;
    Xstart2 = FKinSpace(M,Slist,thetastart1);

    %%%%%%%%%%% Xend  수정 %%%%%%%%%%%%
    %thetaend1 = [0; -50*pi/180; -80*pi/180; 0; -55*pi/180; 0];
    %Xend1 = FKinSpace(M,Slist,thetaend1);
    Xend2 = Trans(0,0,-0.25) * Xstart2;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Tf = 3;
    N = 100;
    method = 3;

    %%%%%%%%%%%%%%%%%%%%%%%%%

    % Trajectory making
    traj2 = CartesianTrajectory(Xstart2, Xend2, Tf, N, method);

    thetalist0 = thetaend;

    eomg = 0.00001;
    ev = 0.00001;    

    % For Trajectory 1

    traj2thetalist = zeros(6,N);  % Trajectory를 따라 움직이게 될 행렬

    % Singularity 를 고려해줘야 함. 2 * pi를 넘어서는 각도를 보정해줄 필요가 존재 >> -pi ~ pi안으로 들어오도록
    for i = 1:N
        % create trajectory using IKinSpace.m

        T = cell2mat(traj2(i));         % cell 형식을 matrix로 바꿔줌
        [thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev);


        if success == 1
            for j = 1:6
                if abs(thetalist0(j)-thetalist(j)) > 0.01
                    if thetalist(j) > 0
                        thetalist0(j) = mod(thetalist(j),2*pi);                           

                        if thetalist0(j) > pi
                            thetalist0(j) = mod(thetalist(j),2*pi) - 2 * pi;                            
                        end
                    else % thetalist(j) < 0    
                        thetalist0(j) = mod(thetalist(j),2*pi) - 2 * pi;
                        %disp('spin');
                        if thetalist0(j) < -pi
                            thetalist0(j) = thetalist0(j) + 2 * pi;
                        end
                    end
                else
                    thetalist0(j) = thetalist(j);
                end
            end
            traj2thetalist(:, i) = thetalist0;
        else
            traj2thetalist(:, i) = thetalist0;
            disp(i);
        end
    end
        joint_pos_mat2 = traj2thetalist;


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
    pause(1);
    
    %% closing the gripper
    
    [~, connector] = sim.simxGetObjectHandle(clientID, 'BarrettHand_attachPoint', sim.simx_opmode_blocking);
    [~, objectSensor] = sim.simxGetObjectHandle(clientID, 'BarrettHand_attachProxSensor', sim.simx_opmode_blocking);
    
    % close the gripper
    sim.simxSetIntegerSignal(clientID, convertStringsToChars(gripperName+"_close"), 1, sim.simx_opmode_blocking);
       
    pause(3);  
    
    %% traj 3
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% 다시 올라가는 trajectory3
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%         thetastart2 = [0; -50*pi/180; -80*pi/180; 0; -55*pi/180; 0]; 
%         Xstart2 = FKinSpace(M,Slist,thetastart2);
    Xstart3 = Xend2;
    %%%%%%%%%%% Xend  수정 %%%%%%%%%%%%
%         thetaend2 = [0; -40*pi/180; -80*pi/180; 0; -65*pi/180; 0];
%         Xend2 = FKinSpace(M,Slist,thetaend2);
    Xend3 = Trans(0,0.25,0.4) * Xstart3;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Tf = 3;
    N = 100;
    method = 3;

    %%%%%%%%%%%%%%%%%%%%%%%%%

    % Trajectory making
    traj3 = CartesianTrajectory(Xstart3, Xend3, Tf, N, method);

    thetalist0 = traj2thetalist(:,100); %[0; -50*pi/180; -80*pi/180; 0; -55*pi/180; 0];

    eomg = 0.00001;
    ev = 0.00001;    

    % For Trajectory 1

    traj3thetalist = zeros(6,N);  % Trajectory를 따라 움직이게 될 행렬

    % Singularity 를 고려해줘야 함. 2 * pi를 넘어서는 각도를 보정해줄 필요가 존재 >> -pi ~ pi안으로 들어오도록
    for i = 1:N
        % create trajectory using IKinSpace.m

        T = cell2mat(traj3(i));         % cell 형식을 matrix로 바꿔줌
        [thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev);


        if success == 1
            for j = 1:6
                if abs(thetalist0(j)-thetalist(j)) > 0.01
                    if thetalist(j) > 0
                        thetalist0(j) = mod(thetalist(j),2*pi);                           

                        if thetalist0(j) > pi
                            thetalist0(j) = mod(thetalist(j),2*pi) - 2 * pi;                            
                        end
                    else % thetalist(j) < 0    
                        thetalist0(j) = mod(thetalist(j),2*pi) - 2 * pi;
                        disp('spin');
                        if thetalist0(j) < -pi
                            thetalist0(j) = thetalist0(j) + 2 * pi;
                        end
                    end
                else
                    thetalist0(j) = thetalist(j);
                end
            end
            traj3thetalist(:, i) = thetalist0;
        else
            traj3thetalist(:, i) = thetalist0;
            disp(i);
        end
    end
        joint_pos_mat3 = traj3thetalist;


    for i=1:n
        tstart = tic;
        for j=1:6
            sim.simxSetJointTargetPosition(clientID, h(j), joint_pos_mat3(j, i), sim.simx_opmode_streaming);
        end
        dt = toc(tstart);
        if dt<duration
            pause(duration-dt);
        end

    end
    pause(2);
    
    %% move in line
    n = 100;
    duration = 0.05;
    
    speed = 1;
    
    wheel_ang_vel1 = [linspace(0, -speed, n/2); linspace(0, -speed, n/2);];
    wheel_ang_vel2 = [linspace(-speed, 0, n/2); linspace(-speed, 0, n/2);];
    
    wheel_ang_vel = [wheel_ang_vel1 wheel_ang_vel2];
    
    for i=1:n
        tstart = tic;
        
        for j=1:2
            sim.simxSetJointTargetVelocity(clientID, wheelHandle(j), wheel_ang_vel(j,i), sim.simx_opmode_streaming);
        end
        
        dt = toc(tstart);
        if dt<duration
            pause(duration-dt);
        end
    end
    
    for j=1:2
        sim.simxSetJointTargetVelocity(clientID, wheelHandle(j), 0, sim.simx_opmode_streaming);
    end
    
    %% draw half circle
    n = 148;
    duration = 0.05;
    
    speed = 1;
    
    wheel_ang_rot1 = [linspace(0, -8/5, n/2); linspace(0, -4/5, n/2);];
    wheel_ang_rot2 = [linspace(-8/5, 0, n/2); linspace(-4/5, 0, n/2);];
    
    wheel_ang_rot = [wheel_ang_rot1 wheel_ang_rot2];
    
    for i=1:n
        tstart = tic;
        
        for j=1:2
            sim.simxSetJointTargetVelocity(clientID, wheelHandle(j), wheel_ang_rot(j,i), sim.simx_opmode_streaming);
        end
        dt = toc(tstart);
        if dt<duration
            pause(duration-dt);
        end
    end
    pause(3);
    
    
    %% %%%%%%%%%%%%%%%%%%%%%%%% Trajectory 4 %%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% 내려놓는 위치로 이동 %%%%%%%%%%%%(하강x)

    % default dt
    duration = 0.01;
    % # of times steps
    n = 100;
    
    thetastart4 = traj3thetalist(:,100);
    thetaend4 = [-90*pi/180; -10*pi/180; -80*pi/180; 0*pi/180; -85*pi/180; 0];

    Tf = 3;
    N = 100;
    method = 3;

    traj4 = JointTrajectory(thetastart4, thetaend4, Tf, N, method);

    for i=1:n
        tstart = tic;
        for j=1:6
            sim.simxSetJointTargetPosition(clientID, h(j), traj4(i, j), sim.simx_opmode_streaming);
        end
        dt = toc(tstart);
        if dt<duration
            pause(duration-dt);
        end
    end
    pause(1);



    %% %%%%%%%%%%%%%% Trajectory 5 %%%%%%%%%%%%%%%%%%%%% 하강

    thetastart5 = [-90*pi/180; -10*pi/180; -80*pi/180; 0*pi/180; -85*pi/180; 0]; 
    Xstart5 = FKinSpace(M,Slist,thetastart5);

    %%%%%%%%%%% Xend  수정 %%%%%%%%%%%%
    Xend5 = Trans(0,0,-0.15) * Xstart5;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Tf = 3;
    N = 100;
    method = 3;

    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Trajectory making
    traj5 = CartesianTrajectory(Xstart5, Xend5, Tf, N, method);

    thetalist0 = [-90*pi/180; -30*pi/180; -90*pi/180; 0*pi/180; -63*pi/180; 0];

    eomg = 0.00001;
    ev = 0.00001;    

    % For Trajectory 1

    traj5thetalist = zeros(6,N);  % Trajectory를 따라 움직이게 될 행렬

    % Singularity 를 고려해줘야 함. 2 * pi를 넘어서는 각도를 보정해줄 필요가 존재 >> -pi ~ pi안으로 들어오도록
    for i = 1:N
        % create trajectory using IKinSpace.m

        T = cell2mat(traj5(i));         % cell 형식을 matrix로 바꿔줌
        [thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev);


        if success == 1
            for j = 1:6
                if abs(thetalist0(j)-thetalist(j)) > 0.01
                    if thetalist(j) > 0
                        thetalist0(j) = mod(thetalist(j),2*pi);                           

                        if thetalist0(j) > pi
                            thetalist0(j) = mod(thetalist(j),2*pi) - 2 * pi;                            
                        end
                    else % thetalist(j) < 0    
                        thetalist0(j) = mod(thetalist(j),2*pi) - 2 * pi;
                        disp('spin');
                        if thetalist0(j) < -pi
                            thetalist0(j) = thetalist0(j) + 2 * pi;
                        end
                    end
                else
                    thetalist0(j) = thetalist(j);
                end
            end
            traj5thetalist(:, i) = thetalist0;
        else
            traj5thetalist(:, i) = thetalist0;
            disp(i);
        end
    end
        joint_pos_mat5 = traj5thetalist;


    for i=1:n
        tstart = tic;
        for j=1:6
            sim.simxSetJointTargetPosition(clientID, h(j), joint_pos_mat5(j, i), sim.simx_opmode_streaming);
        end
        dt = toc(tstart);
        if dt<duration
            pause(duration-dt);
        end

    end
    pause(1);
    
    % open the gripper
    sim.simxSetIntegerSignal(clientID, convertStringsToChars(gripperName+"_close"), 0, sim.simx_opmode_blocking);
    
    pause(3);
    
    %% %%%%%%%%%%%%%%%% Trajectory 6 %%%%%%%%%%%%%%% 물건 내려두고 다시 상승

    Xstart6 = Xend5;
    %%%%%%%%%%% Xend  수정 %%%%%%%%%%%%
    %thetaend2 = [0; -40*pi/180; -80*pi/180; 0; -65*pi/180; 0];
    %Xend2 = FKinSpace(M,Slist,thetaend2);
    Xend6 = Trans(0,0,0.15) * Xstart6;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Tf = 3;
    N = 100;
    method = 3;

    %%%%%%%%%%%%%%%%%%%%%%%%%

    % Trajectory making
    traj6 = CartesianTrajectory(Xstart6, Xend6, Tf, N, method);

    thetalist0 = [0; -50*pi/180; -80*pi/180; 0; -55*pi/180; 0];

    eomg = 0.00001;
    ev = 0.00001;    

    % For Trajectory 1

    traj6thetalist = zeros(6,N);  % Trajectory를 따라 움직이게 될 행렬

    % Singularity 를 고려해줘야 함. 2 * pi를 넘어서는 각도를 보정해줄 필요가 존재 >> -pi ~ pi안으로 들어오도록
    for i = 1:N
        % create trajectory using IKinSpace.m

        T = cell2mat(traj6(i));         % cell 형식을 matrix로 바꿔줌
        [thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev);


        if success == 1
            for j = 1:6
                if abs(thetalist0(j)-thetalist(j)) > 0.01
                    if thetalist(j) > 0
                        thetalist0(j) = mod(thetalist(j),2*pi);                           

                        if thetalist0(j) > pi
                            thetalist0(j) = mod(thetalist(j),2*pi) - 2 * pi;                            
                        end
                    else % thetalist(j) < 0    
                        thetalist0(j) = mod(thetalist(j),2*pi) - 2 * pi;
                        disp('spin');
                        if thetalist0(j) < -pi
                            thetalist0(j) = thetalist0(j) + 2 * pi;
                        end
                    end
                else
                    thetalist0(j) = thetalist(j);
                end
            end
            traj6thetalist(:, i) = thetalist0;
        else
            traj6thetalist(:, i) = thetalist0;
            disp(i);
        end
    end
        joint_pos_mat6 = traj6thetalist;


    for i=1:n
        tstart = tic;
        for j=1:6
            sim.simxSetJointTargetPosition(clientID, h(j), joint_pos_mat6(j, i), sim.simx_opmode_streaming);
        end
        dt = toc(tstart);
        if dt<duration
            pause(duration-dt);
        end

    end
    pause(2);
    
    
    
else
    disp('Failed connecting to remote API server');
end
sim.delete(); % call the destructor!
disp('Program ended');