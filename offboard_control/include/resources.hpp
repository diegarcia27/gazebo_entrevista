// Parameters
float speed_kp = 1.;
float max_speed = 5.0;
float acceptance_radius = 2.0;
float acceleration_kp = 0.6;
float acceleration_ki = 0.03;
float acceleration_kd = 0.;
float thrust_divider = 5000;
float max_acceleration = 2.0;
float min_acceleration = -3.0;


enum OffboardControlMode {
    SHUTDOWN = -1,
    OFF = 0,
    POSITION = 1,
    SPEED = 2,
    THRUST = 3
};

// Very simple function to calculate the distance between two points
// Not very accurate, but good enough for this example 
double calculate_position_error(double lat1, double lon1, double lat2, double lon2) {
    // Calculate the distance between two points
    double R = 6371000; // Radius of the earth in m
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

// Calculate the direction in degrees from north
// ROS has its own built in functions to do this, but I do not know them 
double calculate_direction(double lat1, double lon1, double lat2, double lon2) {
    lat1 = lat1 * M_PI / 180.0;
    lon1 = lon1 * M_PI / 180.0;
    lat2 = lat2 * M_PI / 180.0;
    lon2 = lon2 * M_PI / 180.0;
    double dLon = (lon2 - lon1);

    return (M_PI / 2.0) -atan2(sin(dLon) * cos(lat2),
                 cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon));
}