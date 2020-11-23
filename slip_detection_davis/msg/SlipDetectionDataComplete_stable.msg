std_msgs/Header header

# grasped tight or loose
string condition

# object material plastic or metal
string object

# Detector type Harris or Canny
string detector

# Frame in millisec 5,10,15
uint16 frame_ms


# raw noise
float32 raw_noise_events

# Grasping data for millisec frame, grasp_condiion = opening or closing
string grasp_status 
uint16 grasp_start
float32 grasp_count
float32 grasp_noise_frame
float32 grasp_edge_frame
float32 grasp_corner_frame
float32 grasp_raw_total_events_frame
uint16 grasp_stop
float32 grasp_duration_SS
std_msgs/Float64MultiArray grasping_info_arr

# slippage data for millisec frame
uint16 slip_start
float32 slip_count
float32 slip_count_total
float32 slip_noise_frame
float32 slip_edge_frame
float32 slip_corner_frame
float32 slip_total_events_noise
float32 slip_total_events_edge
float32 slip_total_events_corner
uint16 slip_stop
float32 slip_duration_SS
std_msgs/Float64MultiArray slippage_info_arr

# F/T  sensor data
geometry_msgs/WrenchStamped f_max_data
geometry_msgs/WrenchStamped t_max_data

# Gripper sensor data 0 (closed) -100 (opened)
float32 gripper_percent
