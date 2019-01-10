setupobstacle; % loads obs
kuka = NewKUKAGripper();
kuka_forces = NewKUKAForces();
vel = 0.04;
%% 4.1 Initial motion planning simulation
Rd = [[0 0 1]' [0 -1 0]' [1 0 0]'];
P0 = [370; -440; 150];
P1 = [370; -440; 32];
P2 = [750; -220; 225];
P3 = [620; 350; 225];
H0 = Hom(Rd, P0);
H1 = Hom(Rd, P1);
H2 = Hom(Rd, P2);
H3 = Hom(Rd, P3);
q0 = inverseKUKA(H0, kuka);
q1 = inverseKUKA(H1, kuka);
q2 = inverseKUKA(H2, kuka);
q3 = inverseKUKA(H3, kuka);
%%
setHome(vel);
setGripper(1); % open gripper
%%
setAngles(q0, vel); % move just above the target
%%
setAngles(q1, vel) % move directly to p1 w/o obstacle avoidance
%%
setGripper(0); % close gripper
%%
% Path plan from p1 to p3
t0 = posixtime(datetime('now'));
deltaT = 10;
t1 = t0 + deltaT;
[qref, qpath] = motionplan(q1', q3', t0, t1, kuka_forces, obs, 0.05, 0.01, 0.01);
while posixtime(datetime('now')) < t1
    q = ppval(qref, posixtime(datetime('now')));
    setAngles(q, vel);
end

%%
% This is a test to see if date
t0 = posixtime(datetime('now'));
deltaT = 10;
t1 = t0 + deltaT;
[qref, qpath] = motionplan(q1', q3', t0, t1, kuka_forces, obs, 0.05, 0.01, 0.01);
while posixtime(datetime('now')) < t1
    q = ppval(qref, posixtime(datetime('now')))
end

    