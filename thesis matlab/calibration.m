clear all;

Lidar_Cam = [    0.5784   -0.8157         0 -0.0456 ;
    0.8157    0.5784         0  -0.0075;
    0         0    1.0000 -0.015;
    0  0 0 1];

Robot_Lidar = [1 0 0 -0.032;
    0 1 0 0;
    0 0 1 0.172;
    0 0 0 1];

Robot_Cam = Robot_Lidar*Lidar_Cam;

eul = [0, 0, 0.9539747289];
rotmXYZ = eul2rotm(eul,'XYZ');




