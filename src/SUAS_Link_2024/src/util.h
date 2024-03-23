#ifndef UTIL_H
#define UTIL_H

class Drop_State {
    public:
        Drop_State();
        double lat, lon, heading;
};

Drop_State calc_des_drop_state(double* wind_speed, Drop_State drop_state, Drop_State* des_drop_state);


double metersToLatitude(double meters);
double metersToLongitude(double meters, double latitude);
double latitudeToMeters(double latitude);
double longitudeToMeters(double longitude, double latitude);
double distance(double lat1, double lon1, double lat2, double lon2);

#endif