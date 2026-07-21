# SUPER -> upper layer: waypoint window consumption progress.
uint32 batch_id        # matches LocalGoalSet.batch_id of the consumed window
uint8  consumed_count  # how many waypoints of this batch have been consumed (including skipped)
uint8  active_idx      # index of the waypoint currently being tracked
uint8  skipped_mask    # bitmask of waypoints dropped (reprojection failed / infeasible)
bool   all_consumed    # true when the whole batch is done (final waypoint reached)
