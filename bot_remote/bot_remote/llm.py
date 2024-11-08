# web_interface.py
from flask import Flask, request, render_template, jsonify
import cohere
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from bot_msgs.action import Temp  # Custom action message

app = Flask(__name__)

# Initialize Cohere API
co = cohere.Client('KrjlzkbdVrg5h3M8rWxTl5U9egMJHgw5Md7yIZPr')

# ROS 2 node and action client setup
rclpy.init()
node = Node("web_cohere_interface")
action_client = ActionClient(node, Temp, "task_server")

# Few-shot learning examples for Cohere
FEW_SHOT_EXAMPLES = """
Task: Pick
Input: "Pick the pen"
Output: Pick

Input: "Grab the pen"
Output: Pick

Input: "Take the object"
Output: Pick

Task: Sleep
Input: "Turn off"
Output: Sleep

Input: "Go to sleep"
Output: Sleep

Input: "Shut down"
Output: Sleep

Task: Wake
Input: "Wake up"
Output: Wake

Input: "Turn on"
Output: Wake

Input: "Start the robot"
Output: Wake
"""

# Function to use Cohere's model to determine intent
def get_cohere_intent(user_input):
    prompt = FEW_SHOT_EXAMPLES + f"\nInput: \"{user_input}\"\nOutput: "
    
    response = co.generate(
        model='command-xlarge',
        prompt=prompt,
        max_tokens=10
    )
    
    generated_text = response.generations[0].text.strip().lower()

    if 'pick' in generated_text:
        return 1  # Pick task
    elif 'sleep' in generated_text:
        return 0  # Sleep task
    elif 'wake' in generated_text:
        return 2  # Wake task
    else:
        return None  # Unrecognized intent

# Function to send goal to ROS 2
def send_ros_goal(task_number):
    goal = Temp.Goal()
    goal.task_number = task_number
    action_client.send_goal_async(goal)

# Web page route
@app.route('/')
def index():
    return render_template('index.html')

# Route to process input from webpage
@app.route('/process_input', methods=['POST'])
def process_input():
    user_input = request.json['user_input']
    
    task_number = get_cohere_intent(user_input)
    if task_number is not None:
        send_ros_goal(task_number)
        return jsonify({"status": "Success", "task": task_number})
    else:
        return jsonify({"status": "Error", "message": "Could not recognize the intent"}), 400

if __name__ == "__main__":
    app.run(debug=True)
