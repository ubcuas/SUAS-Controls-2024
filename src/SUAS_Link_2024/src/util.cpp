#include "util.h"
#include "params.h"
#include "linker.h"

Drop_State::Drop_State() {
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
 * @param wind_speed Passed by pointer, plz don't modify
 * @param drop_state Communicated by software team from Pi
 * @param des_drop_state
 */
Drop_State calc_des_drop_state(double* wind_speed, Drop_State drop_state, Drop_State* des_drop_state) {

    // TODO: assuming heading is degrees CW from north
    // x is North (lat), y is East (lon)
    double x_offset = wind_speed[0] * DRIFT_FACTOR + AIRCRAFT_SPEED * cos(drop_state.heading * M_PI/180.0) * RELEASE_DELAY;
    double y_offset = wind_speed[1] * DRIFT_FACTOR + AIRCRAFT_SPEED * sin(drop_state.heading * M_PI/180.0) * RELEASE_DELAY;

    des_drop_state->lat = drop_state.lat - metersToLatitude(x_offset);
    des_drop_state->lon = drop_state.lon - metersToLongitude(y_offset, des_drop_state->lat);
    
}
