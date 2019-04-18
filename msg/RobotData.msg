int32                   id
Point2d                 center
Point2d                 direction
Point2d[]               corners
Point2d[]               path
Point2d                 actual_point
Point2d                 next_point
float32                 angle_to_actual_point
float32                 actual_angle
int32[]                 sector
Point2d                 finish_heading_point
int32                   angle_to_heading_point
geometry_msgs/Point     real_world_position

Wheels_pair             wheels_pair

bool                    path_created
bool                    move
bool                    rotation
bool                    stop

bool                    on_finish_point
bool                    on_finish_heading
bool                    connection_mode
bool                    fine_tune_connection
bool                    ready_to_connect

