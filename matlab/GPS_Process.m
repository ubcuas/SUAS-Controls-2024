lose all
clear all
% Define the file path
file_path = 'incomingdata.txt'; % Replace with your file path
 
% Read the data into a table
GPS_data = ReadParsedGPS(file_path);
Data_Length = height(GPS_data);
 
% Reference and new coordinates
R_lat = GPS_data.Latitude(1); % Reference latitude
R_long = GPS_data.Longitude(1); % Reference longitude
R_alt = GPS_data.Altitude_GPS(1); % Reference Altitude
 
% Make arrays to store the X and Y position moved
X_pos = zeros(1, Data_Length)';
Y_pos = zeros(1, Data_Length)';
Z_pos = zeros(1, Data_Length)';
 
% Iterate through all the data points
for i = 1:Data_Length
    % Current point's coordinates and altitude
    C_lat = GPS_data.Latitude(i);
    C_long = GPS_data.Longitude(i);
    C_alt = GPS_data.Altitude(i);
 
    % Calculate X and Y distances (flat Earth approximation)
    X_pos(i) = distanceBetween(R_lat, R_long, R_lat, C_long);
    Y_pos(i) = distanceBetween(R_lat, R_long, C_lat, R_long);
    
    % Check direction (East/West for X, North/South for Y)
    if C_long < R_long
        X_pos(i) = -X_pos(i);
    end
    if C_lat < R_lat
        Y_pos(i) = -Y_pos(i);
    end
 
    % Calculate Z distance (altitude difference)
    Z_pos(i) = C_alt - R_alt;
end
 
% 3D plot of the path
figure;
plot3(X_pos, Y_pos, Z_pos, 'o-');
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('3D Path of GPS Data');
grid on;
% Set axis limits
xlim([min(X_pos)-10, max(X_pos)+10]); % Extend limits by 10 meters on each side for X
ylim([min(Y_pos)-10, max(Y_pos)+10]); % Extend limits by 10 meters on each side for Y
zlim([-100, 100]); % Extend limits by 10 meters on each side for Z
figure;
plot(X_pos,Y_pos);