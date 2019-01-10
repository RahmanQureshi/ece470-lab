% q0, q2 column vectors representing initial and final positions
function qref=motionplan(q0,q2,t1,t2,myrobot,obs,tol)
    N = 5000;
    alpha = 0.01;
    %q = zeros(N, 6);
    %q(1, 1:6) = q0;
    q = [q0];
    i = 1;
    numObstacles = length(obs);
    while i < N
        q_current = q(end, 1:6);
        Fatt = att(q_current, q2, myrobot);
        Frep = 0;
        if numObstacles == 1
            Frep = Frep + rep(q_current, myrobot, obs);
        else
            for p = 1:numObstacles
                Frep = Frep + rep(q_current, myrobot, obs{p});
            end
        end
        q_next = q_current + alpha*(Fatt' + Frep'); % transpose Frep because rep gives col vector
        q = [q; q_next];
        if norm(wrapTo2Pi(q_next(1:5))-wrapTo2Pi(q2(1:5))) < tol
            display("q-qfinal within tolerance! Exiting");
            break
        end
        i = i + 1;
    end
    t = linspace(t1,t2,size(q,1));
    qref = spline(t,q');
end