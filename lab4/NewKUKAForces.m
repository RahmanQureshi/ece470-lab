% This initializes a KUKA robot with a6=0mm for force calculations
function myrobot = NewKUKA()
    a = [25 315 35 0 0 0];
    alpha = [pi/2 0 pi/2 -pi/2 pi/2 0];
    d = [400 0 0 365 0 161.44];
    theta = [0 0 0 0 0 0];
    DH = [alpha' a' theta' d'];
    n = size(DH, 1);
    links = Link.empty(n, 0);
    for i = 1:n
        links(i) = Link('d', DH(i, 4), 'a', DH(i, 2), 'alpha', DH(i, 1));
    end
    myrobot = SerialLink(links, 'name', 'mykuka');
end