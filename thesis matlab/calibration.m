clear all;

Lidar_Cam = [    0.9999   -0.0150         0 -0.0450 ;
    0.0150    0.9999         0  -0.0052;
    0         0    1.0000 -0.015;
    0  0 0 1];

Robot_Lidar = [0.9998 -0.0176 0 -0.0457;
    0.0176 0.9998 0 -0.004;
    0 0 1 0.172;
    0 0 0 1];

Robot_Cam = Robot_Lidar*Lidar_Cam;

eul = [0, 0, 0.0176];
rotmXYZ = eul2rotm(eul,'XYZ');




