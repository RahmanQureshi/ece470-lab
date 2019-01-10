function myrobot = NewPUMA560()
    a = [0 43.228 0 0 0 0];
    alpha = [pi/2 0 pi/2 -pi/2 pi/2 0];
    d = [76 -23.65 0 43.18 0 20];
    theta = [0 0 0 0 0 0];
    DH = [alpha' a' theta' d'];
    n = size(DH, 1);
    links = Link.empty(n, 0);
    for i = 1:n
        links(i) = Link('d', DH(i, 4), 'a', DH(i, 2), 'alpha', DH(i, 1));
    end
    myrobot = SerialLink(links, 'name', 'mypuma560');
end