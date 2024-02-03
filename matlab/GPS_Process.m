close all
clear all
% Define the file path
file_path = '../RecordedData/Sensor_Data_MarineDrive_Field.csv'; % Replace with your file path

%load the data
DataLocation = file_path;
SampleRate = 57.0;
Gravity = 9.809;
RecordedData = parseData(SampleRate, DataLocation);
RecordedData = rmmissing(RecordedData);
% If abs of Latitdue and longitude is less than 10 then remove the row
RecordedData = RecordedData(abs(RecordedData.Latitude) > 10, :);
RecordedData = RecordedData(abs(RecordedData.Longitude) > 10, :);
% if any acceleration is greater than 10.0 then remove the row
RecordedData = RecordedData(abs(RecordedData.Acc_X) < 10, :);
RecordedData = RecordedData(abs(RecordedData.Acc_Y) < 10, :);
RecordedData = RecordedData(abs(RecordedData.Acc_Z) < 10, :);


Data_Length = height(RecordedData);
SampleRate = 57.0;
P0 = mean(RecordedData.Pressure);

%To visualize the DATA
%Animate3D(RecordedData)
plotAcceleration(RecordedData,1);
plotAltitude(RecordedData,2);

%initialize the vectors and matrices
NumStates = 2;
NumMeasurements = 1;
NumControlInputs = 1;
Dt = 1/SampleRate;
Acc_X_stdev = 0.2 * Gravity; %meters per second squared
Acc_Y_stdev = 0.17 * Gravity; %meters per second squared
Acc_Z_stdev = 0.16 * Gravity; %meters per second squared
Barometer_stdev = 1.466; %meters
Pressure_stdev = 2.0161;
meanAccX = mean(RecordedData.Acc_X)*Gravity*0;
meanAccY = mean(RecordedData.Acc_Y)*Gravity*0;
meanAccZ = mean(RecordedData.Acc_Z)*Gravity*0;
GPS_stdev = 2.0; %m

%State Transition Matrix -- Need to customize based on you system model
F = [1 Dt;
     0 1];

%Control Input Matrix -- Need to customize based on you system model
G = [0.5 * Dt^2;
     Dt];

%Process Noise Covariance Matrix -- Need to customize based on you system model
Q_ = [0.25 * Dt^4 0.5 * Dt^3;
     0.5 * Dt^3 Dt^2];

Q = Q_ * Acc_Z_stdev^2;
Q_X = Q_ * Acc_X_stdev^2;
Q_Y = Q_ * Acc_Y_stdev^2;
Q_Z = Q_ * Acc_Z_stdev^2;

%Measurement Matrix -- Need to customize based on you system model
H = [1 0];

%Measurement Noise Covariance Matrix -- Need to customize based on you system model
R = GPS_stdev^2; 
R_ = Barometer_stdev^2;

%Process Noise Vector -- Need to customize based on you system model
W = zeros(NumStates, 1);

%Initial State Covariance Matrix -- Need to customize based on you system model
P = [0.001 0;
     0 0.001];

%make a kalman filter objects
myKalmanFilter_inst_X = KalmanFilter(NumStates, NumMeasurements, NumControlInputs, Dt);
myKalmanFilter_inst_Y = KalmanFilter(NumStates, NumMeasurements, NumControlInputs, Dt);
myKalmanFilter_inst_Z = KalmanFilter(NumStates, NumMeasurements, NumControlInputs, Dt);

%initialize the object
myKalmanFilter_inst_X.initializeMatrices(F,G,Q_X,H,R,W,P);
myKalmanFilter_inst_Y.initializeMatrices(F,G,Q_Y,H,R,W,P);
myKalmanFilter_inst_Z.initializeMatrices(F,G,Q_Z,H,R_,W,P);

%Make a vector to store the results
KalmanFilterResults_X = zeros(length(RecordedData.Time),NumStates);
KalmanFilterResults_Y = zeros(length(RecordedData.Time),NumStates);
KalmanFilterResults_Z = zeros(length(RecordedData.Time),NumStates);

% Reference and new coordinates
R_lat = RecordedData.Latitude(1); % Reference latitude
R_long = RecordedData.Longitude(1); % Reference longitude
R_alt = RecordedData.Altitude_GPS(1); % Reference Altitude
 
% Make arrays to store the X and Y position moved
X_pos = zeros(1, Data_Length)';
Y_pos = zeros(1, Data_Length)';
Z_pos = zeros(1, Data_Length)';
 
% Iterate through all the data points to get X, Y and Z
for i = 1:Data_Length
    % Current point's coordinates and altitude
    C_lat = RecordedData.Latitude(i);
    C_long = RecordedData.Longitude(i);
    C_alt = RecordedData.Altitude_GPS(i);
 
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

%Run the Kalman Filters
for i = 1:length(RecordedData.Time)
    %predict the state
    % Q = Q_ * pow2(Acc_Z_stdev)*pow2(RecordedData.Acc_Z(i)*Gravity);
    myKalmanFilter_inst_X.predict(RecordedData.Acc_Y(i) * Gravity - meanAccX);
    myKalmanFilter_inst_Y.predict(RecordedData.Acc_X(i) * Gravity - meanAccY);
    myKalmanFilter_inst_Z.predict(RecordedData.Acc_Z(i) * Gravity - meanAccZ);

    %update the state every 10th sample
    % if (mod(i,0) == 0)
    %     myKalmanFilter_inst.update(RecordedData.Altitude(i));
    %     myExtendedKalmanFilter_inst.update(RecordedData.Pressure(i));
    % end
    
    % myKalmanFilter_inst_Z.update(Z_pos(i));
    myKalmanFilter_inst_Z.update(RecordedData.Altitude(i));
    myKalmanFilter_inst_X.update(X_pos(i));
    myKalmanFilter_inst_Y.update(Y_pos(i));
    
    %store the results
    KalmanFilterResults_X(i,:) = myKalmanFilter_inst_X.X';
    KalmanFilterResults_Y(i,:) = myKalmanFilter_inst_Y.X';
    KalmanFilterResults_Z(i,:) = myKalmanFilter_inst_Z.X';
end

% 3D plot of the path
figure;
plot3(X_pos, Y_pos, RecordedData.Altitude, 'o-');
hold on
plot3(KalmanFilterResults_X(:,1), KalmanFilterResults_Y(:,1), KalmanFilterResults_Z(:,1), 'o-');
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('3D Path of RAW GPS Data vs Kalman Estimate');
grid on;
% Set axis limits
lim_Size = 50; %meters
xlim([-1*lim_Size, lim_Size]); % Extend limits by 10 meters on each side for X
ylim([-1*lim_Size, lim_Size]); % Extend limits by 10 meters on each side for Y
zlim([-1*lim_Size, lim_Size]); % Extend limits by 10 meters on each side for Z

figure;
% Subplot for X_POS
subplot(2,1,1); % Two rows, one column, first subplot
plot(RecordedData.Time, X_pos, 'b');
hold on;
plot(RecordedData.Time, KalmanFilterResults_X(:,1), 'r');
hold off;
xlabel('Time (s)');
ylabel('XPos (m)');
legend('Measured (GPS)', 'Estimated-MATLAB(LKF)');
title('XPos Estimation Results');

% Subplot for Velocity
subplot(2,1,2); % Two rows, one column, second subplot
plot(RecordedData.Time, KalmanFilterResults_X(:,2), 'r');
hold on
% plot(RecordedData.Time, RecordedData.P_Vel, 'g');
% plot(RecordedData.Time, ExtendedKalmanFilterResults(:,2), 'k');
hold off
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Estimated-MATLAB(LKF)');
title('Velocity Estimation Results');

figure;
% Subplot for Y_POS
subplot(2,1,1); % Two rows, one column, first subplot
plot(RecordedData.Time, Y_pos, 'b');
hold on;
plot(RecordedData.Time, KalmanFilterResults_Y(:,1), 'r');
hold off;
xlabel('Time (s)');
ylabel('YPos (m)');
legend('Measured (GPS)', 'Estimated-MATLAB(LKF)');
title('YPos Estimation Results');

% Subplot for Velocity
subplot(2,1,2); % Two rows, one column, second subplot
plot(RecordedData.Time, KalmanFilterResults_Y(:,2), 'r');
hold on
% plot(RecordedData.Time, RecordedData.P_Vel, 'g');
% plot(RecordedData.Time, ExtendedKalmanFilterResults(:,2), 'k');
hold off
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Estimated-MATLAB(LKF)');
title('Velocity Estimation Results');

figure;
% Subplot for Z_POS
subplot(2,1,1); % Two rows, one column, first subplot
plot(RecordedData.Time, RecordedData.Altitude, 'b');
hold on;
plot(RecordedData.Time, KalmanFilterResults_Z(:,1), 'r');
hold off;
xlabel('Time (s)');
ylabel('ZPos (m)');
legend('Measured (Barometer)', 'Estimated-MATLAB(LKF)');
title('ZPos Estimation Results');

% Subplot for Velocity
subplot(2,1,2); % Two rows, one column, second subplot
plot(RecordedData.Time, KalmanFilterResults_Z(:,2), 'r');
hold on
% plot(RecordedData.Time, RecordedData.P_Vel, 'g');
% plot(RecordedData.Time, ExtendedKalmanFilterResults(:,2), 'k');
hold off
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Estimated-MATLAB(LKF)', 'Estimated ESP (Barometer)');
title('Velocity Estimation Results');

%to view the data in motion
% Animate3D(KalmanFilterResults_X(:,1), KalmanFilterResults_Y(:,1), KalmanFilterResults_Z(:,1), 1/SampleRate)

% Convert exsiting X,Y to Lat and Long
% make a table to store the results
Lat = zeros(1, Data_Length)';
Long = zeros(1, Data_Length)';
KalmanFilter_LatLongTable = table(Lat, Long);

% Iterate through all the data points to get Lat and Long
for i = 1:Data_Length
    %convert the current X and Y to Lat and Long
    [KalmanFilter_LatLongTable.Lat(i), KalmanFilter_LatLongTable.Long(i)] = xytoLatLong(KalmanFilterResults_X(i,1), KalmanFilterResults_Y(i,1), R_lat, R_long);
end

%plot GPS Cordinates on Map
figure;
geoplot(RecordedData.Latitude,RecordedData.Longitude, "b-^")
hold on
geoplot(KalmanFilter_LatLongTable.Lat,KalmanFilter_LatLongTable.Long, "r-.")
geobasemap satellite
title("Position Map");
legend("Standalone GPS Data", "MATLAB-LKF");