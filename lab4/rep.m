% Written by Yoonsun You
% Modifications:
% 1) take finiteness of cylinders into account (Rahman Qureshi)
% 2) add repelling force from z=32mm plane (Rahman Qureshi)
function tau = rep(q,myrobot,obs)
    tau = zeros(6,1);
    
    for i = 1:6
        H_cur = forward(q',myrobot,i); 
        o_i = H_cur(1:3,4); %o_i0 of current position
        x = o_i(1);
        y = o_i(2);
        z = o_i(3);
        
        %Initialize Jacobian 
        J = Jvoi(myrobot, q, i);
        
        %For the cylinder obstacle type
        if obs.type == 'cyl'
            R = obs.R;
            if z <= obs.h
                b_pre = obs.c + R*(o_i(1:2) - obs.c)/norm(o_i(1:2) - obs.c);
                %b is closest point on obstacle to current joint
                b = [b_pre; o_i(3)];
                rho = o_i - b;
            else % z > obs.h
                if norm(o_i(1:2) - obs.c) <= R % point is right over top of cylinder
                    b = [o_i(1:2); obs.h];
                    rho = o_i - b;
                else % projection onto top rim
                    xc = obs.c(1); % center of cylinder x
                    yc = obs.c(2); % center of cylinder y
                    theta = atan2(y-yc, x-xc);
                    b = [xc + R*cos(theta); yc + R*sin(theta); obs.h];
                    rho = o_i - b;
                end
            end
        %For the sphere object type
        elseif obs.type == 'sph'
            %b is closest point on obstacle to current joint
            b = obs.c + obs.R*(o_i - obs.c)/norm(o_i - obs.c);
            rho = o_i - b;
        elseif obs.type == 'pla'
            b = [o_i(1:2); obs.h];
            rho = o_i - b;
        end

        %Repulsive force only exists if we are within a certain distance
        %rho0 of obstacle
        if norm(rho) <= obs.rho0
            F_rep = (1/norm(rho) - 1/obs.rho0)*(1/norm(rho)^3)*rho;
        else
            %If outside of range, set repulsive force to zero
            F_rep = zeros(3,1);
        end

        %Compute tau
        tau = tau + J'*F_rep;
    end
    
    %Normalize tau (only if magnitude isn't zero)
    if norm(tau) ~= 0
        tau = tau/norm(tau);
    end
end