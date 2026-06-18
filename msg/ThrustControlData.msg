uint64 timestamp    # time since system start (microseconds)

float32[4] thrust_error
float32[4] thrust_control_out
float32[4] thrust_desired
float32[4] thrust_desired_dot
float32[4] thrust_control_out_pwm
float32[4] thrust_feedforward_out
float32[4] thrust_feedback_out
float32[4] thrust_kalman_filter_control_out_pwm
float32[4] motor_speed_dot
float32[4] motor_speed
float32[4] f_x
float32[4] g_x
float32[4] deta_t

float32 thrust_start

float32 kp
float32 ki
float32 kd

float32 bet_freestream_m_s
float32 bet_alpha_deg
float32 bet_yaw_rate_rad_s
float32 bet_length_m
float32[4] bet_achieved_lift_n
int32[4] bet_status
int32[4] bet_iterations
uint32[4] bet_lookup_time_us

# TOPICS thrust_control_data
