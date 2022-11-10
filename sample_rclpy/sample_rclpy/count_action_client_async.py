import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sample_msgs.action import Count

class CountActionClient(Node):
    def __init__(self):
        super().__init__('count_action_client')
        self._action_client = ActionClient(self, Count, 'count')
    
    async def send_goal(self, max_count=10):
        goal_msg = Count.Goal()
        goal_msg.max_count = max_count

        self._action_client.wait_for_server()

        goal_handle = await self._action_client.send_goal_async(goal_msg,
            feedback_callback=self.feedback_callback)
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
        self.get_logger().info('Goal accepted :)')
        
        result = await goal_handle.get_result_async()
        self.get_logger().info(f'Result: {result.result.result}')
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.current_count}')

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor(2)
    action_client = CountActionClient()
    executor.add_node(action_client)
    executor.create_task(action_client.send_goal, 10)
    executor.spin()

if __name__ == '__main__':
    main()