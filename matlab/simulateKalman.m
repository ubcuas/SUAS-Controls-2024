%script to simulate the Kalman filter
close all;
clear all;

%load the data
DataLocation = '../RecordedData/Data_LinAcc_Altitude_Z_Axis_With_Kalman_OUTPUT_Stationary_1';
SampleRate = 55.0;
Gravity = 9.809;
RecordedData = parseData(SampleRate, DataLocation);
SampleRate = 55.0;

%To visualize the DATA
%Animate3D(RecordedData)
plotAcceleration(RecordedData,1);
plotAltitude(RecordedData,2);

%initialize the vectors and matrices
NumStates = 2;
NumMeasurements = 1;
NumControlInputs = 1;
Dt = 1/SampleRate;
Acc_Z_stdev = 0.0030 * Gravity; %meters per second squared
Barometer_stdev = 1.5; %meters
meanAccZ = mean(RecordedData.Acc_Z)*Gravity;

%State Transition Matrix -- Need to customize based on you system model
F = [1 Dt;
     0 1];

%Control Input Matrix -- Need to customize based on you system model
G = [0.5 * Dt^2;
     Dt];

%Process Noise Covariance Matrix -- Need to customize based on you system model
Q = [0.25 * Dt^4 0.5 * Dt^3;
     0.5 * Dt^3 Dt^2];

Q = Q * Acc_Z_stdev^2;

%Measurement Matrix -- Need to customize based on you system model
H = [1 0];

%Measurement Noise Covariance Matrix -- Need to customize based on you system model
R = Barometer_stdev^2;

%Process Noise Vector -- Need to customize based on you system model
W = zeros(NumStates, 1);

%Initial State Covariance Matrix -- Need to customize based on you system model
P = [0.001 0;
     0 0.001];

%make a kalman filter object
myKalmanFilter_inst = KalmanFilter(NumStates, NumMeasurements, NumControlInputs, Dt);

%initialize the object
myKalmanFilter_inst.initializeMatrices(F,G,Q,H,R,W,P);

%Make a vector to store the results
KalmanFilterResults = zeros(length(RecordedData.Time),NumStates);

%Run the Kalman Filter
for i = 1:length(RecordedData.Time)
    %predict the state
    myKalmanFilter_inst.predict(RecordedData.Acc_Z(i) * Gravity - meanAccZ);
    
    %update the state every 10th sample
    if (mod(i,5) == 0)
        myKalmanFilter_inst.update(RecordedData.Altitude(i));
    end
    % myKalmanFilter_inst.update(RecordedData.Altitude(i));
    
    %store the results
    KalmanFilterResults(i,:) = myKalmanFilter_inst.X';
end

% Plot Altitude and Velocity in Subplots
figure(3);
% Subplot for Altitude
subplot(2,1,1); % Two rows, one column, first subplot
plot(RecordedData.Time, RecordedData.Altitude, 'r');
hold on;
plot(RecordedData.Time, KalmanFilterResults(:,1), 'b');
plot(RecordedData.Time, RecordedData.P_Pos, 'g');
hold off;
xlabel('Time (s)');
ylabel('Altitude (m)');
legend('Measured', 'Estimated-MATLAB','Estimated-ESP' );
title('Altitude Estimation Results');

% Subplot for Velocity
subplot(2,1,2); % Two rows, one column, second subplot
plot(RecordedData.Time, KalmanFilterResults(:,2), 'r');
hold on
plot(RecordedData.Time, RecordedData.P_Vel, 'b');
hold off
xlabel('Time (s)');
ylabel('Velocity (m/s)');
legend('Estimated-MATLAB', 'Estimated-ESP');
title('Velocity Estimation Results');
