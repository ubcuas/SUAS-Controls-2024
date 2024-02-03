function distance = distanceBetween(lat1, long1, lat2, long2)
    % Convert degrees to radians
    lat1 = deg2rad(lat1);
    long1 = deg2rad(long1);
    lat2 = deg2rad(lat2);
    long2 = deg2rad(long2);

    % Calculate differences
    deltaLong = long2 - long1;
    sdlong = sin(deltaLong);
    cdlong = cos(deltaLong);

    % Sine and cosine of latitudes
    slat1 = sin(lat1);
    clat1 = cos(lat1);
    slat2 = sin(lat2);
    clat2 = cos(lat2);

    % Haversine formula
    delta = (clat1 * slat2) - (slat1 * clat2 * cdlong);
    delta = delta^2;
    delta = delta + (clat2 * sdlong)^2;
    delta = sqrt(delta);
    denom = (slat1 * slat2) + (clat1 * clat2 * cdlong);
    delta = atan2(delta, denom);

    % Convert to distance
    distance = delta * 6372795;
end