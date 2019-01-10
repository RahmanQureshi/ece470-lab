% computes the velocity jacobian of the ith link
function J=Jvoi(robot, q, i)
    n = length(robot.a);
    isrevolute = robot.links.isrevolute;
    J = zeros(3,n);
    Hn = forward(q, robot, i); % calculate final position of ith link
    On = Hn(1:3, 4);
    for j=1:i
        Hi = forward(q, robot, j-1);
        Oi = Hi(1:3,4);
        Zi = Hi(1:3,3);
        if isrevolute(i)
            J(1:3, j) = cross(Zi, On-Oi);
        else
            J(1:3, j) = Zi;
        end
    end
end