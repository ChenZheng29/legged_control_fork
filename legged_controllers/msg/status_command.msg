bool locomotion_enable     # true:mpc and wbc control   false:position control

# mpc and wbc control
string gait                  # e.g. "trot" "flying_trot" "pace" "stance"

# position control
int32 stage                # 0:damping  1:squat  2:stance
