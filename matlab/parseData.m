function result =  parseData(samplingFrequency, filePath)
    % Read the CSV file into a table
    data = readtable(filePath);

    %remove the row with NaN
    data = rmmissing(data);

    % Extract columns for Altitude, Acc_X, Acc_Y, and Acc_Z
    Altitude = data.Altitude;
    Acc_X = data.Acc_X;
    Acc_Y = data.Acc_Y;
    Acc_Z = data.Acc_Z;
    P_Pos = data.P_Pos;
    P_Vel = data.P_Vel;
    Latitude = data.Latitude;
    Longitude = data.Longitude;
    Altitude = data.Altitude;
    q_w = data.q_w;
    q_x = data.q_x;
    q_y = data.q_y;
    q_z = data.q_z;
    Altitude_GPS = data.Altitude_GPS;
    % Alt_1 = data.Alt_1;
    Pressure = data.Pressure;

    % Calculate the number of data points
    numDataPoints = height(data);

    % Create a time vector based on the sampling frequency
    % Assuming the first sample is at t=0
    Time = (0:numDataPoints-1)' / samplingFrequency;


    % Combine all the data into a single table (optional)
    result = table(Time, Altitude, Acc_X, Acc_Y, Acc_Z, P_Pos, P_Vel, Pressure, Latitude, Longitude, q_w, q_x, q_y, q_z, Altitude_GPS);  %add alt_1 if needed
end