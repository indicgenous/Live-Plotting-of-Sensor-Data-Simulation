import rospy
from std_msgs.msg import String
import socket
import json
import threading

# Configure the socket server
HOST = "0.0.0.0"  # Update this with your server IP
PORT = 7005
BACKLOG = 1

def handle_client(conn, addr):
    """Handle a single client connection."""
    rospy.loginfo(f"Client connected: {addr}")

    def callback(data):
        """Callback for ROS topic subscription."""
        try:
            json_data = json.loads(data.data)
            conn.sendall(json.dumps(json_data).encode("utf-8") + b"\n")
        except Exception as e:
            rospy.logerr(f"Error sending data: {e}")
   
    rospy.Subscriber("sensor_data", String, callback)
    rospy.spin()  # Keep the node running

    conn.close()
    rospy.loginfo(f"Connection closed: {addr}")

def socket_server():
    """Main socket server function."""
    rospy.init_node("ros_to_socket_server", anonymous=True)

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((HOST, PORT))
    server_socket.listen(BACKLOG)
    rospy.loginfo(f"Socket server running on {HOST}:{PORT}")

    while not rospy.is_shutdown():
        try:
            conn, addr = server_socket.accept()
            threading.Thread(target=handle_client, args=(conn, addr)).start()
        except socket.error as e:
            rospy.logerr(f"Socket error: {e}")
        except rospy.ROSInterruptException:
            break
        finally:
            server_socket.close()

if __name__ == "__main__":
    try:
        socket_server()
    except rospy.ROSInterruptException:
        pass

