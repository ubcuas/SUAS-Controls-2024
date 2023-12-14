function Animate3D(X, Y, Z, delay)
% Extract acceleration data
Acc_X = X;
Acc_Y = Y;
Acc_Z = Z;

numPoints = length(Acc_Z);

%getting delay
% delay = RecordedDataTime(2) - RecordedData.Time(1);
% Assuming you have Acc_X, Acc_Y, Acc_Z, and numPoints
N = 200; % Number of points to display in the trail

% Prepare the figure
figure;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Movement Animation');
xlim([min(Acc_X) max(Acc_X)]);
ylim([min(Acc_Y) max(Acc_Y)]);
zlim([5*min(Acc_Z) 5*max(Acc_Z)]);
view(3);
hold on;

% Create plot objects
hPoint = plot3(Acc_X(1), Acc_Y(1), Acc_Z(1), 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'b');
hLine = plot3(Acc_X(1:min(N,numPoints)), Acc_Y(1:min(N,numPoints)), Acc_Z(1:min(N,numPoints)), 'b');

% Initialize text annotation
hText = text(min(Acc_X), max(Acc_Y), max(Acc_Z), '', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right');

% Loop through each time step and update the plot
for k = 1:numPoints
    % Update point
    set(hPoint, 'XData', Acc_X(k), 'YData', Acc_Y(k), 'ZData', Acc_Z(k));

    % Update line
    idxStart = max(1, k-N+1);
    set(hLine, 'XData', Acc_X(idxStart:k), 'YData', Acc_Y(idxStart:k), 'ZData', Acc_Z(idxStart:k));

    % Update text
    pointInfo = sprintf('Time: %.2f\nX: %.2f\nY: %.2f\nZ: %.2f', k*delay, Acc_X(k), Acc_Y(k), Acc_Z(k));
    set(hText, 'String', pointInfo, 'Position', [Acc_X(k) Acc_Y(k) Acc_Z(k)]);

    drawnow;
    pause(delay/3.0); % Adjust as needed
end
end