# Parsed raw IMU data from Novatel OEM6 receiver
Header header

NovatelMessageHeader novatel_msg_header

uint32 gps_week_num
float64 gps_seconds

# All measurements in this message are instantaneous values;
# accelerations are in m/s
int32 z_acceleration
int32 y_acceleration
int32 x_acceleration

# attitude rate is in radians
int32 yaw_rate
int32 roll_rate
int32 pitch_rate
