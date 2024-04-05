#ifndef UTIL_H
#define UTIL_H

#include "Sender.h"

class Drop_State { // deprecated
    public:
        Drop_State();
        double lat, lon, heading;
};

void calc_des_drop_state(double windspeed, double wind_heading, struct_message drop_data, struct_message* des_drop_data);


double metersToLatitude(double meters);
double metersToLongitude(double meters, double latitude);
double latitudeToMeters(double latitude);
double longitudeToMeters(double longitude, double latitude);
double distance(double lat1, double lon1, double lat2, double lon2);

#endif