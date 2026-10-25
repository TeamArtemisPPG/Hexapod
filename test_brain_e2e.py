#!/usr/bin/env python3
"""
test_brain_e2e.py — End-to-end integration test for the Autonomous Hexapod Brain.
"""

import os
import sys
import time
import subprocess

workspace = '/home/vishal/gz_hex'
secrets_file = os.path.join(workspace, '.brain_secrets')
if os.path.isfile(secrets_file):
    with open(secrets_file) as f:
        for line in f:
            line = line.strip()
            if line.startswith('NVIDIA_API_KEY='):
                os.environ['NVIDIA_API_KEY'] = line.split('=', 1)[1].strip()

ros_env = os.environ.copy()
ros_env['PYTHONPATH'] = f"/home/vishal/.local/lib/python3.12/site-packages:{ros_env.get('PYTHONPATH', '')}"

query_runner_code = """
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys

rclpy.init()
node = Node('test_query_runner')
pub = node.create_publisher(String, '/brain/query', 10)
received = []

def on_resp(msg):
    print(f'[RESPONSE RECEIVED] {msg.data}')
    received.append(msg.data)

sub = node.create_subscription(String, '/brain/response', on_resp, 10)
print('Test runner initialized, waiting for connections...')

for _ in range(15):
    rclpy.spin_once(node, timeout_sec=0.1)

print('Sending query: Rotate 30 degrees to inspect the room')
pub.publish(String(data='Rotate 30 degrees to inspect the room'))

t0 = time.time()
while time.time() - t0 < 30 and not received:
    rclpy.spin_once(node, timeout_sec=0.2)

print(f'Total responses received: {len(received)}')
for r in received:
    print(f'>>> Response: {r}')

rclpy.shutdown()
sys.exit(0 if len(received) > 0 else 1)
"""

with open('/tmp/test_query_runner.py', 'w') as f:
    f.write(query_runner_code)

procs = []
try:
    print("Launching sensor_bridge...")
    p_sensor = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /home/vishal/gz_hex/install/setup.bash && ros2 run rover sensor_bridge"],
        cwd=workspace, env=ros_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    procs.append(p_sensor)

    print("Launching tool_dispatcher...")
    p_tool = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /home/vishal/gz_hex/install/setup.bash && ros2 run rover tool_dispatcher"],
        cwd=workspace, env=ros_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    procs.append(p_tool)

    print("Launching nvidia_brain...")
    p_brain = subprocess.Popen(
        ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /home/vishal/gz_hex/install/setup.bash && ros2 run rover nvidia_brain"],
        cwd=workspace, env=ros_env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    )
    procs.append(p_brain)

    time.sleep(5)  # Let nodes spin up

    print("Executing query runner...")
    res = subprocess.run(
        ["bash", "-c", "source /opt/ros/jazzy/setup.bash && source /home/vishal/gz_hex/install/setup.bash && python3 /tmp/test_query_runner.py"],
        cwd=workspace, env=ros_env, capture_output=True, text=True, timeout=60
    )
    print("Runner STDOUT:\n", res.stdout)
    if res.stderr:
        print("Runner STDERR:\n", res.stderr)

    if res.returncode == 0:
        print("✅ SUCCESS: Autonomous brain end-to-end test passed!")
    else:
        print(f"❌ Test failed with code {res.returncode}")

finally:
    for p in procs:
        p.terminate()
        try:
            p.wait(timeout=2)
        except subprocess.TimeoutExpired:
            p.kill()
