uint8      track_id
uint32     hr_collisions    # In step it is either 0 (false) or 1 (true), in episde --> #collisions with human track_id
float32    hr_dist
float32[]  hr_dists         # Valid only in episode --> gives min, max and average distances
