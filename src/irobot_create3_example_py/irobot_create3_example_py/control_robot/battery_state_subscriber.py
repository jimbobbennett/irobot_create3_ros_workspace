from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, LivelinessPolicy, DurabilityPolicy

from sensor_msgs.msg import BatteryState

class BatteryStateSubscriber(Node):
    '''
    This is a ROS node that subscribes to the /battery_state message published by the iRobot
    Create3 robot. This is an example subscriber to show how to subscribe to messages.

    All nodes must derive from Node to implement the required ROS functionality.

    Topics publish messages that can be typed, and most message types are in the sensor_msgs.msg package.
    For example, the battery state is of type sensor_msgs.msg.BatteryState.

    When subscribing, the subscriber must specify the quality of service (QoS).
    This MUST match the QoS provided by the publisher. If not, then no messages will be recieved.

    You can get the type and QoS by running this command:

    ros2 topic info -v <topic>

    Where topic is the topic, such as /battery_state. The output will give you the type and QoS profile.
    You can also uses this command to see how many nodes are subscribing to that topic.

    The output for the battery state is:

    Type: sensor_msgs/msg/BatteryState

    Publisher count: 1

    Node name: robot_state
    Node namespace: /
    Topic type: sensor_msgs/msg/BatteryState
    Endpoint type: PUBLISHER
    GID: 01.0f.34.90.5c.09.ad.d2.01.00.00.00.00.01.4f.03.00.00.00.00.00.00.00.00
    QoS profile:
        Reliability: BEST_EFFORT
        Durability: VOLATILE
        Lifespan: 9223372036854775807 nanoseconds
        Deadline: 9223372036854775807 nanoseconds
        Liveliness: AUTOMATIC
        Liveliness lease duration: 9223372036854775807 nanoseconds

    Subscription count: 0

    This tells you that the type is sensor_msgs/msg/BatteryState, so sensor_msgs.msg.BatteryState.
    The QoS values you care about are the reliability, durability and liveliness. The subscription MUST
    match these.
    '''

    def __init__(self):
        '''
        Initializes the subscriber
        '''

        # Initialize the class using the base constructor and provide a name
        # This name is used for logging
        super().__init__('battery_state_subscriber')

        # Log that we are subscribing
        self.get_logger().info('Subscribing to battery state')

        # The QoS values for the battery_sate are:
        # QoS profile:
        #   Reliability: BEST_EFFORT
        #   Durability: VOLATILE
        #   Lifespan: 9223372036854775807 nanoseconds
        #   Deadline: 9223372036854775807 nanoseconds
        #   Liveliness: AUTOMATIC
        #   Liveliness lease duration: 9223372036854775807 nanoseconds
        #
        # We don't care about the times, but do care about Reliability, Durability and Liveliness
        # Create a QoS profile that matches these values
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            liveliness=LivelinessPolicy.AUTOMATIC,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # Create a subscriber on the /battery_state topic.
        # To create a subscription you need to specify the type, in this case
        # sensor_msgs.msg.BatteryState, the topic name, the function to call whenever a message
        # is received, and the QoS profile.
        self.subscription_battery_state = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.get_battery_state,
            qos_profile)

        # Log that we have subscribed successfully
        self.get_logger().info('Subscribed!')

    def get_battery_state(self, msg: BatteryState):
        '''
        The callback called whenever a message is published on /battery_state.
        '''

        # Log the current battery percentage.
        self.get_logger().info(
            'Battery percentage:{:.0%}'.format(msg.percentage))
