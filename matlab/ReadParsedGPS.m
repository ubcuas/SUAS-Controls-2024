function GPSDATA = ReadParsedGPS(file_location)
    % format long g
    % Read the data into a table
    opts = detectImportOptions(file_location, 'Delimiter', ',');
%     opts = setvartype(opts, {'latitude', 'longitude', 'altitude_m_', 'course', 'speed_km_h_', 'hdop' }, 'double');
    GPSDATA = readtable(file_location, opts);
end
    