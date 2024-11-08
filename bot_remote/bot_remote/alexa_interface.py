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
Input: pick the pen
Output: pick

Input: take the pen
Output: pick

Input: hey intelli, Grab the pencil.
Output: pick

Input: hey intelli, Collect the marker.
Output: pick

Input: hey intelli, Can you take the marker.
Output: pick

Input: hey intelli, Grab the pencil.
Output: pick

Input: hey, intelli take the pen
Output: pick

Input: turn off the robot
Output: sleep

Input: hey intelli, Power down the robot
Output: sleep

Input: hey intelli, Power down
Output: sleep

Input: hey intelli, shut down
Output: sleep

Input: hey intelli, Let the system rest
Output: sleep

Input: hey intelli, go to initial mode
Output: sleep

Input: robot sleep now
Output: sleep

Input: Can you turn off now
Output: sleep

Input: hey, robot wake-up
Output: wake

Input: hey intelli wake-up
Output: wake

Input: hey intelli Begin operation
Output: wake

Input: wake up
Output: wake

Input: activate the intelli
Output: wake 

Input: awake the intelliarm
Output: wake

Input: awake the intelli
Output: wake
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

# Define perform_task_based_on_input to handle the input and process the task
def perform_task_based_on_input(user_input):
    # Get intent from cohere
    task_number = get_cohere_intent(user_input)

    if task_number is not None:
        # Send the task goal to ROS 2
        send_ros_goal(task_number)
        return task_number
    else:
        return "Unknown task"

# Web page route
@app.route('/')
def index():
    return render_template('index.html')

# Route to process input from webpage
@app.route('/process_input', methods=['POST'])
def process_input():
    try:
        # Access JSON data
        data = request.get_json()
        user_input = data['user_input']
        
        # Process the input and get the task
        task = perform_task_based_on_input(user_input)

        # Respond with the task that was performed
        return jsonify({"task": task}), 200
    
    except KeyError:
        return jsonify({"message": "Invalid input key."}), 400
    except Exception as e:
        return jsonify({"message": str(e)}), 500

if __name__ == "__main__":
    app.run(debug=True)
