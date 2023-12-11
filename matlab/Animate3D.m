function Animate3D(RecordedData)
% Extract acceleration data
Acc_X = RecordedData.Acc_X;
Acc_Y = RecordedData.Acc_Y;
Acc_Z = RecordedData.Acc_Z;

numPoints = length(Acc_Z);

%getting delay
delay = RecordedData.Time(2) - RecordedData.Time(1);
% Assuming you have Acc_X, Acc_Y, Acc_Z, and numPoints
N = 20; % Number of points to display in the trail

% Prepare the figure
figure;
grid on;
xlabel('Acc_X');
ylabel('Acc_Y');
zlabel('Acc_Z');
title('3D Movement Animation');
xlim([min(Acc_X) max(Acc_X)]);
ylim([min(Acc_Y) max(Acc_Y)]);
zlim([min(Acc_Z) max(Acc_Z)]);
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
    pointInfo = sprintf('Time: %.2f\nX: %.2f\nY: %.2f\nZ: %.2f', k, Acc_X(k), Acc_Y(k), Acc_Z(k));
    set(hText, 'String', pointInfo, 'Position', [Acc_X(k) Acc_Y(k) Acc_Z(k)]);

    drawnow;
    pause(delay); % Adjust as needed
end
end