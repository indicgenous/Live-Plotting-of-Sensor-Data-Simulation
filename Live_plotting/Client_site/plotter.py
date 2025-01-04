import socket
import json
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Configure the socket client
HOST = "192.168.21.157"  # Replace with the IP of the machine running the server
PORT = 7005         # The same port number used in the server

# Constants
MAX_POINTS = 100  # Maximum number of points to keep in the plot
SOCKET_TIMEOUT = 10  # Timeout for socket connection

# Initialize data buffers
timestamps, accelerations, velocities, distances = [], [], [], []

def fetch_data():
    """Connect to the socket server and yield data."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
        client_socket.settimeout(SOCKET_TIMEOUT)  # Set timeout for connection
        try:
            client_socket.connect((HOST, PORT))
            buffer = b""
            while True:
                try:
                    data = client_socket.recv(1024)  # Receive data in chunks
                    if not data:
                        print("No data received. Connection closed by server.")
                        break

                    buffer += data
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        try:
                            yield json.loads(line.decode("utf-8"))  # Convert the data into a dictionary
                        except json.JSONDecodeError as e:
                            print(f"Error decoding JSON: {e}")
                except socket.timeout:
                    print("Socket timed out.")
                    break
        except Exception as e:
            print(f"Error connecting to server: {e}")

def update_graph(frame, lines, axes, data_gen):
    """Update the graphs with the latest data."""
    try:
        data = next(data_gen)
        print(f"Received data: {data}")  # Debugging: Print received data

        # Ensure that the necessary fields are present
        for field in ["timestamp", "acceleration", "velocity", "distance"]:
            if field not in data:
                print(f"Error: Missing field '{field}' in data: {data}")
                return lines

        timestamps.append(data["timestamp"])
        accelerations.append(data["acceleration"])
        velocities.append(data["velocity"])
        distances.append(data["distance"])

        # Keep only the last MAX_POINTS points
        if len(timestamps) > MAX_POINTS:
            timestamps.pop(0)
            accelerations.pop(0)
            velocities.pop(0)
            distances.pop(0)

        # Update plots
        for idx, (line, y_data) in enumerate(zip(lines, [accelerations, velocities, distances])):
            line.set_data(timestamps, y_data)
            axes[idx].set_xlim(min(timestamps), max(timestamps))
            axes[idx].set_ylim(min(y_data) - 10, max(y_data) + 10)
    except StopIteration:
        print("Data generator stopped.")
    except Exception as e:
        print(f"Error updating graph: {e}")

    return lines

# Set up Matplotlib figure and subplots
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
fig.suptitle("Live Sensor Data: Acceleration, Velocity, and Distance")

# Configure subplots
for ax, title, ylabel in zip(
    axes,
    ["Acceleration", "Velocity", "Distance"],
    ["Acceleration (m/s²)", "Velocity (m/s)", "Distance (m)"]
):
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid()
axes[-1].set_xlabel("Time")

# Create line objects for each subplot
lines = [
    axes[0].plot([], [], color="blue", label="Acceleration")[0],
    axes[1].plot([], [], color="green", label="Velocity")[0],
    axes[2].plot([], [], color="red", label="Distance")[0]
]

# Set up Matplotlib animation
data_gen = fetch_data()
ani = animation.FuncAnimation(fig, update_graph, fargs=(lines, axes, data_gen), interval=1000)

# Show the plot
plt.tight_layout()
plt.show()
