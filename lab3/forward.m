% Returns homogenous matrix representing position and orientation of
% the ith frame.
function H = forward(joint, robot, i)
    if i==0
        H = Hom(eye(3),[0;0;0]);
    else
        H = DH(robot.a(1:i), robot.alpha(1:i), robot.d(1:i), joint);
    end
end