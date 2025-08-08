import rospy
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

def switch_controllers(start_controllers, stop_controllers, timeout=5.0):
    """
    switch ROS controllers.
    """
    rospy.wait_for_service('/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy(
            '/controller_manager/switch_controller', SwitchController)
        req = SwitchControllerRequest()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        req.strictness = 1  # STRICT
        req.start_asap = True
        req.timeout = timeout
        resp = switch_controller(req)
        return resp.ok
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return False

def impedance_control_mode():
    success = switch_controllers(
        start_controllers=["cartesian_impedance_example_controller"],
        stop_controllers=["position_joint_trajectory_controller"]
    )
    if not success:
        rospy.logerr("Failed to switch to impedance control!")
        return

def position_control_mode():
    success = switch_controllers(
        start_controllers=["position_joint_trajectory_controller"],
        stop_controllers=["cartesian_impedance_example_controller"]
    )
    if not success:
        rospy.logerr("Failed to switch to position joint trajectory control!")
        return
