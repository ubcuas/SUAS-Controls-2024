%script to simulate the Kalman filter
close all;
clear all;

%load the data
DataLocation = '../RecordedData/3DKalmanFilterDemo1.csv';
SampleRate = 57.0;

PosTable = readtable(DataLocation);

Animate3D(PosTable.Px, PosTable.Py, PosTable.Pz, 1/57);

figure;

plot3(PosTable.Px, PosTable.Py, PosTable.Pz);
title('3D Trajectory');
grid on;
xlabel("X (m)");
ylabel("Y (m)");
zlabel("Z (m)");


