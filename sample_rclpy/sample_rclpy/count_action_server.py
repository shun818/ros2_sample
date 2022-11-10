import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sample_msgs.action import Count

class CountActionServer(Node):
    def __init__(self):
        super().__init__('count_action_server')
        self._action_server = ActionServer(
            self,
            Count,
            'count',
            self.execute_callback,
            cancel_callback=self.cancel_callback)
        self.goal_handle = None
    
    def execute_callback(self, goal_handle : ServerGoalHandle):
        self.get_logger().info('Executing goal ...')

        feedback_msg = Count.Feedback()
        rate = self.create_rate(1.0)
        self.goal_handle = goal_handle
        for i in range(goal_handle.request.max_count):
            if goal_handle.is_cancel_requested:
                break
            rate.sleep()
            feedback_msg.current_count = i + 1
            goal_handle.publish_feedback(feedback_msg)
        
        result = Count.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.result = False
            return result
        else:
            goal_handle.succeed()
            result.result = True
            return result
    
    def cancel_callback(self, goal_handle):
        if self.goal_handle == goal_handle:
            return CancelResponse.ACCEPT
        else:
            return CancelResponse.REJECT

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(2)
    node = CountActionServer()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
