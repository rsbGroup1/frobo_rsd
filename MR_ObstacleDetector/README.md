# obstacle avoidance for RSD Group 1
Parameters: threshold_ignore(default 0.2), threshold_slow (default 0.5), threshold_stop(default 1.2),laser_scan (default=/scan)
Input: Laser Range Data
Output: 2 bool topics (slow, stop) indicating if there is an object in range.
