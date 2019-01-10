% Written by Yoonsun You
function tau = rep(q,myrobot,obs)
    tau = zeros(6,1);
    
    for i = 1:6
        H_cur = forward(q',myrobot,i); 
        o_i = H_cur(1:3,4); %o_i0 of current position
        
        %Initialize Jacobian 
        J = Jvoi(myrobot, q, i);
        
        %For the cylinder obstacle type
        if obs.type == 'cyl'
            obs.c;
            o_i(1:2);
            obs.R;
            b_pre = obs.c + obs.R*(o_i(1:2) - obs.c)/norm(o_i(1:2) - obs.c);
            %b is closest point on obstacle to current joint
            b = [b_pre; o_i(3)];
            rho = norm(o_i - b);
            %Calculate the gradient
            gradient = (o_i - b)/rho;
        %For the sphere object type
        elseif obs.type == 'sph'
            %b is closest point on obstacle to current joint
            b = obs.c + obs.R*(o_i - obs.c)/norm(o_i - obs.c);
            rho = norm(o_i - b);
            %Calculate the gradient
            gradient = (o_i - b)/rho;
        end

        %Repulsive force only exists if we are within a certain distance
        %rho0 of obstacle
        if rho <= obs.rho0
            F_rep = (1/rho - 1/obs.rho0)*(1/rho^2)*gradient;
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