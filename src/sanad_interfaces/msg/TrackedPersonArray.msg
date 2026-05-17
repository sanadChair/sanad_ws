# Array of all persons currently being tracked in one frame.
# Published at camera frame rate (up to the detector's processing speed).

std_msgs/Header header

# All active tracks in this frame. May be empty if no person is detected.
TrackedPerson[] tracks

# ID of the designated follow target. -1 = no target selected yet.
# Set by the human_follow_goal node after target acquisition.
int32 target_id