import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
import requests # For making API calls
import json
import os

# Assuming a custom message type for robot commands, e.g., 'MoveArmToPose'
# from my_robot_msgs.msg import MoveArmToPose

class VLAIntegrationNode(Node):
    def __init__(self):
        super().__init__('vla_integration_node')
        self.declare_parameter('vla_api_key', 'YOUR_OPENAI_API_KEY') # Store API key securely
        self.vla_api_key = self.get_parameter('vla_api_key').get_parameter_value().string_value
        if self.vla_api_key == 'YOUR_OPENAI_API_KEY':
            self.get_logger().error("VLA API Key not set. Please set the 'vla_api_key' parameter or environment variable.")
            rclpy.shutdown()
            return
        self.vla_api_url = "https://api.openai.com/v1/chat/completions" # Example OpenAI API

        self.user_command_sub = self.create_subscription(
            String,
            '/user_command',
            self.command_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.latest_image_msg = None

        # Publisher for robot actions (e.g., to a MoveIt action client or simple movement commands)
        # self.robot_action_pub = self.create_publisher(MoveArmToPose, '/robot_action_commands', 10)

        self.get_logger().info('VLA Integration Node started. Waiting for commands and images...')

    def image_callback(self, msg):
        self.latest_image_msg = msg

    def command_callback(self, msg):
        user_command = msg.data
        self.get_logger().info(f"Received command: '{user_command}'")

        if self.latest_image_msg is None:
            self.get_logger().warn("No image received yet. Cannot process VLA command.")
            return

        # Convert ROS Image message to OpenCV image
        try:
            # Assume rgb8 encoding for simplicity, adjust as needed
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Encode image to base64 for API transmission
        _, buffer = cv2.imencode('.jpg', cv_image)
        encoded_image = base64.b64encode(buffer).decode('utf-8')

        # Construct payload for VLA API (e.g., OpenAI GPT-4o)
        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self.vla_api_key}"
        }
        payload = {
            "model": "gpt-4o", # Or other VLA model
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": f"The user wants me to '{user_command}'. What are the precise robot actions needed to achieve this goal, given the current scene? Respond with a JSON object containing 'action_type' and 'parameters'. Example: {{'action_type': 'move_to_pose', 'parameters': {{'x': 0.1, 'y': 0.2, 'z': 0.3, 'qx':0.0, 'qy':0.0, 'qz':0.0, 'qw':1.0}}}} or {{'action_type': 'grasp', 'parameters': {{'object_id': 'red_block'}}}}. If no action is possible, say 'no_action'."},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{encoded_image}"}}
                    ]
                }
            ],
            "max_tokens": 300
        }

        try:
            response = requests.post(self.vla_api_url, headers=headers, json=payload)
            response.raise_for_status() # Raise an exception for HTTP errors
            vla_response = response.json()
            
            # Extract and parse the VLA's suggested action
            action_text = vla_response['choices'][0]['message']['content']
            self.get_logger().info(f"VLA suggested: {action_text}")

            self.process_vla_action(action_text)

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"VLA API request failed: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing VLA response: {e}")

    def process_vla_action(self, action_json_string):
        """Parses the VLA's action JSON and publishes ROS 2 commands."""
        try:
            # Attempt to parse as JSON. If it's not JSON, it might be a direct 'no_action' or an error message
            if action_json_string.strip().lower() == 'no_action':
                self.get_logger().info("VLA determined no action is necessary or possible.")
                return

            action_data = json.loads(action_json_string)
            action_type = action_data.get('action_type')
            parameters = action_data.get('parameters', {})

            if action_type == 'move_to_pose':
                # Here, you would publish to a MoveIt action client or similar.
                # Example:
                # move_goal = MoveArmToPose.Goal()
                # move_goal.pose.position.x = parameters.get('x', 0.0)
                # ... populate other pose fields ...
                # self.robot_action_pub.send_goal_async(move_goal)
                self.get_logger().info(f"Publishing move_to_pose command with parameters: {parameters}")
            elif action_type == 'grasp':
                # Here, you would trigger a grasping mechanism, possibly via a ROS 2 service call.
                self.get_logger().info(f"Triggering grasp action for object_id: {parameters.get('object_id')}")
            else:
                self.get_logger().warn(f"Unknown action_type received from VLA: {action_type}")

        except json.JSONDecodeError:
            self.get_logger().error(f"VLA response was not a valid JSON: '{action_json_string}'")
        except Exception as e:
            self.get_logger().error(f"Error in process_vla_action: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VLAIntegrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
