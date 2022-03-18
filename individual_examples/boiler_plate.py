import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data

# Import your message types here!
# WARNING this code is not expected to run as is, edits must be made.

class YourNodeName(Node):

    def __init__(self):
        super().__init__('<your_node_name>')
        self.subscription = self.create_subscription(<msg_type1>, <topic_name1>, self.listener_callback,
                                                        qos_profile_sensor_data)
        
        self.publisher = self.create_publisher(<msg_type2>, <topic_name2>, 10)
        
        self._action_client = ActionClient(self, <msg_type3>, <action_name>)

    def listener_callback(self, msg):
        """
        Executed every time subscription get's a message
        :type msg: <msg_type1>
        :rtype: None
        """
        self.get_logger().info('I heard: "%s"' % msg)
        

def main(args=None):
    rclpy.init(args=args)

    your_node = YourNodeName()
    try:
        rclpy.spin(your_node)
    except KeyboardInterrupt:
        print('Caught keyboard interrupt')
    except BaseException:
        print('Exception:', file=sys.stderr)
    finally:
        print("done")
        your_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
        
'''
For examples of how to publish and send actions in python please refer to the previous example codes
'''