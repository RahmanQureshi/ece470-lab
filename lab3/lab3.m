myrobot = NewPUMA560();
%% Part 1: Attractive Force
%Get initial joints
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4) = 100*[-1;3;3]/4;
q1 = inversePUMA560(H1,myrobot);

%Get final joints
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4) = 100*[3;-1;2]/4;
q2 = inversePUMA560(H2,myrobot);

%Get torques to apply on individual joints
tau = att(q1,q2,myrobot)

%% Part3: Repulsive Force Cylinder
setupobstacle

%Get torque for each joint from repulsive force
q3 = 0.9*q1 + 0.1*q2;
tau = rep(q3,myrobot,obs{1}) %this tests the torque for the cylinder obstacle
%%
setupobstacle
hold on
%Set axes
axis([-100 100 -100 100 0 200])
view(-32,50)
%Plot Obstacle
plotobstacle(obs);
%Get the motion planning with the obstacles
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t = linspace(0,10,300);
q = ppval(qref,t)';
plot(myrobot,q);

hold off