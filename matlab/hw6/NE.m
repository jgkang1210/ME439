% with previous joint's info & next joint's info calculate force, torque
% forwardly and backwardly
function taulist = NE(thetalist, dthetalist, ddthetalist, g, ...
            Mtib, Ftip, m, Ilist, pi_1_c_i, pi_c_i, pi_1_p_i)
    n = size(thetalist, 1);

    % starts from fixed cord
    wi = [0; 0; 0];
    dwi = [0; 0; 0];
    % dpi = [0; 0; 0];
    ddpi = -g;
    
    % every rotation axis is z axis in this prob
    z0 = [0; 0; 1];
    
    % first Fi will be F tib
    Fi = Ftip;
    Ni = Mtib;
    
    % force and moment by forward recursion
    F_hatlist = zeros(3,n);
    N_hatlist = zeros(3,n);
    
    taulist = zeros(n,1);

    % length check
    for i=1:n
        if pi_1_c_i(:,:,i) == pi_c_i(:,:,i) + pi_1_p_i(:,:,i)
            % disp("Check ok");
            % disp(i);
        else
            disp("Please correct your link vectors");
            disp(i);
        end
    end
    
    % create rotation matrix
    R = zeros(3,3,n+1);
    
    R01 = [
            cos(thetalist(1)) -sin(thetalist(1)) 0;
            sin(thetalist(1)) cos(thetalist(1)) 0;
            0 0 1];
        
    R12 = [
            1 0 0;
            0 cos(-thetalist(2)) -sin(-thetalist(2));
            0 sin(-thetalist(2)) cos(-thetalist(2))];
        
    R12 = R12 * [
            cos(-pi/2) 0 sin(-pi/2);
            0 1 0;
            -sin(-pi/2) 0 cos(-pi/2)];
    
    R23 = [
            cos(thetalist(3)) -sin(thetalist(3)) 0;
            sin(thetalist(3)) cos(thetalist(3)) 0;
            0 0 1];
        
    R34 = [
            1 0 0;
            0 cos(thetalist(4)) -sin(thetalist(4));
            0 sin(thetalist(4)) cos(thetalist(4))];
        
    R34 = R34 * [
            cos(pi/2) 0 sin(pi/2);
            0 1 0;
            -sin(pi/2) 0 cos(pi/2)];
        
    R45 = [
            1 0 0;
            0 cos(-thetalist(5)) -sin(-thetalist(5));
            0 sin(-thetalist(5)) cos(-thetalist(5))];
        
    R45 = R45 * [
            cos(-pi/2) 0 sin(-pi/2);
            0 1 0;
            -sin(-pi/2) 0 cos(-pi/2)];
    
    R56 = [
            1 0 0;
            0 cos(thetalist(6)) -sin(thetalist(6));
            0 sin(thetalist(6)) cos(thetalist(6))];
        
    R56 = R56 * [
            cos(pi/2) 0 sin(pi/2);
            0 1 0;
            -sin(pi/2) 0 cos(pi/2)];

    R(:,:,1) = R01;
    R(:,:,2) = R12;
    R(:,:,3) = R23;
    R(:,:,4) = R34;
    R(:,:,5) = R45;
    R(:,:,6) = R56;
    R(:,:,7) = eye(3);
    
    % Acceleration calculation
    for i = 1:n
        wi1 = wi;
        wi = R(:,:,i)'*(wi1 + dthetalist(i)*z0);
        dwi = R(:,:,i)'*(dwi + ddthetalist(i)*z0 ...
            + cross(dthetalist(i)*wi1, z0));
        
        % dpi = dpi + cross(wi, pi_1_p_i(:,:,i));
        ddpi = R(:,:,i)'*ddpi + cross(dwi, pi_1_p_i(:,:,i)) ...
                + cross(wi, cross(wi, pi_1_p_i(:,:,i)));
            
        ddpi_c = ddpi + cross(dwi, pi_c_i(:,:,i)) ...
                + cross(wi, cross(wi, pi_c_i(:,:,i)));
            
        fi_hat = m(i) * ddpi_c;
        ni_hat = Ilist(:, :, i)*dwi + cross(wi, Ilist(:, :, i)*wi);
        
        F_hatlist(:,i) = fi_hat;
        N_hatlist(:,i) = ni_hat;
    end
    
    % Force calculation
    for i = n:-1:1
        Fi1 = Fi;
        Fi = F_hatlist(:, i) + R(:,:,i+1)*Fi1;
        Ni = N_hatlist(:, i) - cross(Fi, (pi_1_p_i(:,:,i)+pi_c_i(:,:,i))) ...
            + R(:,:,i+1)*Ni + cross(R(:,:,i+1)*Fi1, pi_c_i(:,:,i));
        taui = dot(Ni, z0);
        taulist(i) = taui;
    end
end