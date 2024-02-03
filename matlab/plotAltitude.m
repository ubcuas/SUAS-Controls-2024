function plotAltitude(RecordedData, FigNum)
    Time = RecordedData.Time;
    Altitude = RecordedData.Altitude;

    figure(FigNum);

    ScaleFactor = 1.1;

    yMin = min(Altitude) * ScaleFactor;
    yMax = max(Altitude) * ScaleFactor;

    plot(Time, Altitude);

    xlabel('Time (s)');
    ylabel('Altitude (m)');
    title('Altitude vs. Time');
end