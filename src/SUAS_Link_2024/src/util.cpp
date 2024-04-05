#include "util.h"
#include "params.h"
#include "linker.h"

Drop_State::Drop_State() { // Deprecated
    lat = 0;
    lon = 0;
    heading = 0;
}

// Approximations
double metersToLatitude(double meters) { 
    return meters / M_PER_LAT_DEG; 
}

double metersToLongitude(double meters, double latitude) { 
    return meters / (M_PER_LAT_DEG * cos(latitude * M_PI/180.0)); 
}

double latitudeToMeters(double latitude) {
    return latitude * M_PER_LAT_DEG;
}

double longitudeToMeters(double longitude, double latitude) {
    double meters_per_degree_lon = M_PER_LAT_DEG * cos(latitude * PI / 180.0);
    return longitude * meters_per_degree_lon;
}

// Function to calculate the distance between two GPS points (approximation for short distances) using Pythagorean
double distance(double lat1, double lon1, double lat2, double lon2) {
    double dlat_m = latitudeToMeters(lat2 - lat1);
    double dlon_m = longitudeToMeters(lon2 - lon1, (lat1 + lat2)/2); // Use average latitude, probably doesn't make a difference though
    return sqrt(dlat_m*dlat_m + dlon_m*dlon_m);
}

/*
 * Calculate desired drop state
 * @param windspeed
 * @param wind_heading
 * @param drop_state Communicated by software team from Pi
 * @param des_drop_state
 */
void calc_des_drop_state(double windspeed, double wind_heading, struct_message drop_data, struct_message* des_drop_data) {

    // TODO: assuming heading is degrees CW from north
    // x is North (lat), y is East (lon)
    double windspeed_x = windspeed * cos(wind_heading * M_PI/180.0);
    double windspeed_y = windspeed * sin(wind_heading * M_PI/180.0);

    double x_offset = windspeed_x * DRIFT_FACTOR + AIRCRAFT_SPEED * cos(drop_data.heading * M_PI/180.0) * RELEASE_DELAY;
    double y_offset = windspeed_y * DRIFT_FACTOR + AIRCRAFT_SPEED * sin(drop_data.heading * M_PI/180.0) * RELEASE_DELAY;

    des_drop_data->lat = drop_data.lat - metersToLatitude(x_offset);
    des_drop_data->lon = drop_data.lon - metersToLongitude(y_offset, des_drop_data->lat);  
    des_drop_data->heading = drop_data.heading;
    des_drop_data->bottleID = drop_data.bottleID;
}
