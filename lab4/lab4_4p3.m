%% Setup robot object and other global variables
setupobstacle; % loads obs
kuka = NewKUKAGripper();
kuka_forces = NewKUKAForces();
vel = 0.04;
deltaT = 10;

%% Compute states list
Rd = [[0 0 1]' [0 -1 0]' [1 0 0]'];
P0 = [370; -440; 150];
P1 = [370; -440; 32];
P2 = [750; -220; 225];
P3 = [620; 350; 225];
H0 = Hom(Rd, P0);
H1 = Hom(Rd, P1);
H2 = Hom(Rd, P2);
H3 = Hom(Rd, P3);

states{1}.H = H0;
states{1}.close = 0;
states{1}.open = 1;

states{2}.H = H1;
states{2}.close = 1;
states{2}.open = 0;

states{3}.H = H2;
states{3}.close = 0;
states{3}.open = 0;

states{4}.H = H3;
states{4}.close = 0;
states{4}.open = 1;

% Calculate motion planning
n = length(states)
for i=1:n
    q1 = getAngles() % q1 = [0; 0; 0; 0; 0; 0];
    q2 = inverseKUKA(states{i}.H, kuka);
    t0 = 0;
    t1 = t0 + deltaT;
    [qref, qpath] = motionplan(q1', q2', t0, t1, kuka_forces, obs, 0.05, 0.01, 0.01);
    states{i}.qref = qref;
end

%% Execute motion planning
n = length(states)
for i=1:n
    t0 = posixtime(datetime('now'));
    t1 = t0 + deltaT;
    while posixtime(datetime('now')) < t1
        q = ppval(states{i}.qref, posixtime(datetime('now'))-t0);
        setAngles(q, vel);
    end
    if states{i}.close
        setGripper(0)
    end
    if states{i}.open
        setGripper(1)
    end
end
