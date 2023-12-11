function plotAcceleration(RecordedData, FigNum)

Gravity = 9.809;

% Assuming you have vectors Time, Acc_X, Acc_Y, and Acc_Z
Time = RecordedData.Time;
Acc_X = RecordedData.Acc_X * Gravity;
Acc_Y = RecordedData.Acc_Y * Gravity;
Acc_Z = RecordedData.Acc_Z * Gravity;

% Assuming you have vectors Time, Acc_X, Acc_Y, and Acc_Z

% Create a figure
figure(FigNum);

% Calculate common limits for Y-axis based on the overall data range
yMin = min([min(Acc_X), min(Acc_Y), min(Acc_Z)]);
yMax = max([max(Acc_X), max(Acc_Y), max(Acc_Z)]);

ScaleFactor = 1.1;
yMin = yMin*ScaleFactor;
yMax = yMax*ScaleFactor;

% Plot Acc_X
subplot(3, 1, 1);
plot(Time, Acc_X);
title('Acceleration in X');
xlabel('Time');
ylabel('Acc_X');
ylim([yMin, yMax]); % Set the same Y-axis limits

% Plot Acc_Y
subplot(3, 1, 2);
plot(Time, Acc_Y);
title('Acceleration in Y');
xlabel('Time');
ylabel('Acc_Y');
ylim([yMin, yMax]); % Set the same Y-axis limits

% Plot Acc_Z
subplot(3, 1, 3);
plot(Time, Acc_Z);
title('Acceleration in Z');
xlabel('Time');
ylabel('Acc_Z');
ylim([yMin, yMax]); % Set the same Y-axis limits

% Add a main title to the figure
sgtitle('Acceleration Components Over Time (m/s^2 vs seconds)');

% Optional: Adjust subplot spacing
set(gcf, 'Position', get(0, 'Screensize')); % Makes the figure full screen
end