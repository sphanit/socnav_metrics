## Same message for stepwise and episodic metrics ##
uint8   STEP=0
uint8   EPISODE=1    

uint8       type
string      t_name
time        time_stamp
int8        success             #Only valid for episode, otherwise set to -1
uint32      obs_collisions
float32     movement_jerk       
float32     min_movement_jerk   #Only valid for episode, otherwise inf
float32     spl                 #Only valid for episode, otherwise set to -1
HRMetrics[] social_metrics