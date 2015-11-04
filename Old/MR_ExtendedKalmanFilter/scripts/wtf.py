#!/usr/bin/env python

import rospy
from MR_ExtendedKalmanFilter.srv import GetStatus, GetStatusRequest

if __name__ == '__main__':
    rospy.init_node('spawner', anonymous=True)
    print 'looking for node MR_ExtendedKalmanFilter...'
    rospy.wait_for_service('MR_ExtendedKalmanFilter/get_status')

    s = rospy.ServiceProxy('MR_ExtendedKalmanFilter/get_status', GetStatus)
    resp = s.call(GetStatusRequest())
    print resp.status
