function [newLat, newLon] = xyToLatLong(x, y, initialLat, initialLon)
    % Earth’s radius in meters
    R = 6378137;

    % Convert latitude to radians
    initialLatRad = deg2rad(initialLat);

    % Calculate the change in latitude
    dLat = y / R;
    newLat = initialLat + rad2deg(dLat);

    % Calculate the change in longitude, accounting for Earth's curvature
    dLon = x / (R * cos(initialLatRad));
    newLon = initialLon + rad2deg(dLon);
end