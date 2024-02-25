#ifndef UTIL_H
#define UTIL_H

class Drop_State {
    public:
        Drop_State();
        double lat, lon, heading;
};

Drop_State calc_des_drop_state(double* wind_speed, Drop_State drop_state, Drop_State* des_drop_state);

#endif