#include "util.h"
#include "params.h"

Drop_State::Drop_State() {
    lat = 0;
    lon = 0;
    heading = 0;
}

// Approximations
double metersToLatitude(double meters) { return meters / 111320.0; }
double metersToLongitude(double meters, double latitude) { return meters / (111320.0 * cos(latitude * M_PI/180.0)); }

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