close all;
clear all;
RecordedData = readtable('../RecordedData/Triumf_full_kalman_filter_test.csv');
% If abs of Latitdue and longitude is less than 10 then remove the row
RecordedData = RecordedData(abs(RecordedData.Px) < 200, :);
RecordedData = RecordedData(abs(RecordedData.Py) < 200, :);
RecordedData = RecordedData(abs(RecordedData.Pz) < 200, :);
Data_Length = height(RecordedData);
figure;
plot3(RecordedData.Px, RecordedData.Py, RecordedData.Pz);
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('3D Path of ESP Kalman Estimate');
grid on;
% Set axis limits
lim_Size = 100; %meters
xlim([-1*lim_Size, lim_Size]); % Extend limits by 10 meters on each side for X
ylim([-1*lim_Size, lim_Size]); % Extend limits by 10 meters on each side for Y
zlim([-1*lim_Size, lim_Size]); % Extend limits by 10 meters on each side for Z

Lat = zeros(1, Data_Length)';
Long = zeros(1, Data_Length)';
KalmanFilter_LatLongTable = table(Lat, Long);
R_long = RecordedData.Long(1,1);
R_lat = RecordedData.Lat(1,1);

% Iterate through all the data points to get Lat and Long
for i = 1:Data_Length
    %convert the current X and Y to Lat and Long
    [KalmanFilter_LatLongTable.Lat(i), KalmanFilter_LatLongTable.Long(i)] = xytoLatLong(RecordedData.Px(i,1), RecordedData.Py(i,1), R_lat, R_long);
end

%plot GPS Cordinates on Map
figure;
geoplot(RecordedData.Lat,RecordedData.Long, "b-^")
hold on
geoplot(KalmanFilter_LatLongTable.Lat,KalmanFilter_LatLongTable.Long, "r-")
geobasemap satellite
title("Position Map");
legend("Standalone GPS Data", "ESP32-LKF");