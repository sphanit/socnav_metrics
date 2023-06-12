## Same message for stepwise and episodic metrics ##
uint8   STEP=0
uint8   EPISODE=1    

uint8       type
string      t_name
bool        success         #Only valid for episode, otherwise set to -1
time        time_stamp
uint32      obs_collisions
float32     movement_jerk
float32     spl             #Only valid for episode, otherwise set to -1
HRMetrics[] social_metrics