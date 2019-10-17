import rospy

def wait_for_param(
    param_name,
    waiting_node='',
    log_message=True,
    fail_after=None):
    """Get a param, waiting until it is available

    Arguments:
        param_name: the ros parameter name to grab
        waiting_node: optional, the name of the node which is waiting on this
            parameter
        log_message: defaults true, log an info message while waiting
        fail_after: optional, seconds after which to give up and throw an error
            defaults to infinitely long

    Returns:
        param: the requested parameter
    """
    sleep_rate = rospy.Rate(1.0)
    cnt = 0

    if fail_after is not None:
        assert isinstance(fail_after, int), 'fail_after must be integer seconds'

    while not rospy.has_param(param_name) and not rospy.is_shutdown():
        cnt += 1

        if log_message:
            rospy.loginfo(waiting_node + ' waiting on parameter ' + param_name)

        if fail_after is not None:
            assert cnt < fail_after,\
                waiting_node + ' timed out waiting for parameter ' + param_name

        sleep_rate.sleep()

    return rospy.get_param(param_name)

