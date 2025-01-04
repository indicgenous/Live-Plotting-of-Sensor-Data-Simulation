# Updated generator.py
import rospy
from std_msgs.msg import String
import random
import time
import json

# Constants
PUBLISH_RATE = 1  # Publish rate in Hz


def generate_data():
    """Simulates sensor data and publishes it to a ROS topic."""
    rospy.init_node("data_generator", anonymous=True)
    pub = rospy.Publisher("sensor_data", String, queue_size=10)
    rate = rospy.Rate(PUBLISH_RATE)  # Publish rate

    while not rospy.is_shutdown():
        # Generate random sensor data
        data = {
            "timestamp": time.time(),
            "acceleration": random.uniform(-5, 5),
            "velocity": random.uniform(20, 50),
            "distance": random.uniform(100, 200)
        }
        try:
            rospy.loginfo(f"Publishing: {data}")
            pub.publish(json.dumps(data))  # Publish as a JSON string
        except rospy.ROSException as e:
            rospy.logerr(f"Error publishing data: {e}")
        rate.sleep()

if __name__ == "__main__":
    try:
        generate_data()
    except rospy.ROSInterruptException:
        pass