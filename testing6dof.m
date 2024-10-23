L1 = Link('d',0.3,'a',0,'alpha',-pi/2,'qlim',[-2*pi 2*pi]);
L2 = Link('d', 0, 'a', 0.4, 'alpha', 0, 'qlim', [-pi/2, 0]);
L3 = Link('d', 0, 'a', 0.4, 'alpha', 0, 'qlim', [0, pi/2]);
L4 = Link('d', 0, 'a', 0, 'alpha', pi/2, 'qlim', [-pi/4, 5*pi/4]);
L5 = Link('d', 0.2, 'a', 0, 'alpha', -pi/2, 'qlim', [-pi, pi]);
L6 = Link('d', 0.2, 'a', 0, 'alpha', pi/2, 'qlim', [-pi, pi]);
L7 = Link('d', 0.2, 'a', 0, 'alpha', 0, 'qlim', [-2*pi, 2*pi]);


robot = SerialLink([L1, L2, L3, L4, L5, L6, L7], 'name','myRobot');

q = [0, 0, 0, 0,0,0, 0];

scale = 0.2;
workspace = [-0.5 1.5 -0.5 1.5 0 1];

robot.teach(q, 'workspace', workspace, 'scale', scale); 
axis equal