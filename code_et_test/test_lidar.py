from rplidar import RPLidar
import numpy as np
import time
import matplotlib.pyplot as plt

# -----------------------------
# Setup and start the LIDAR
# -----------------------------
lidar = RPLidar("/dev/ttyUSB0", baudrate=256000)
lidar.connect()
print(lidar.get_info())
lidar.start_motor()
time.sleep(1)

# Create an array for 360 degree measurements (in mm)
# We use a list of 360 values initialized to 0.
tableau_lidar_mm = [0] * 360

# Pre-compute angles (in radians) for each degree (0 to 359)
teta = [i * np.pi / 180 for i in range(360)]

# -----------------------------
# Setup the live polar plot
# -----------------------------
plt.ion()  # Turn on interactive mode
fig = plt.figure()
ax = fig.add_subplot(111, projection='polar')
# Initial scatter plot; we use a dummy color array (you can use a colormap if desired)
sc = ax.scatter(teta, tableau_lidar_mm, s=5, c=tableau_lidar_mm, cmap='viridis')
ax.set_rmax(8000)
ax.grid(True)
plt.title("Live LIDAR Scan (Polar Coordinates)")

# -----------------------------
# Main loop: update plot with each scan
# -----------------------------
try:
    # Iterate over LIDAR scans (using the 'express' scan type if available)
    for scan in lidar.iter_scans(scan_type='express'):
        # For each scan, update the measurement for each angle.
        # Each element of scan is a tuple: (quality, angle, distance)
        for quality, angle, distance in scan:
            # Convert the angle to an index between 0 and 359.
            # (Adjust the conversion if your LIDAR reports angles differently.)
            angle_index = int(min(359, max(0, 359 - angle)))
            tableau_lidar_mm[angle_index] = distance

        # Update the scatter plot:
        # Remove the old scatter plot and create a new one.
        # (Alternatively, you could update the offsets if you prefer.)
        sc.remove()
        sc = ax.scatter(teta, tableau_lidar_mm, s=5, c=tableau_lidar_mm, cmap='viridis')

        # Redraw the figure canvas and pause briefly.
        plt.draw()
        plt.pause(0.001)

        # Optionally, print the number of points processed in this scan
        print("Number of points in scan: {}".format(len(scan)))

except KeyboardInterrupt:
    print("Acquisition stopped by user.")

finally:
    # -----------------------------
    # Stop the LIDAR and close the connection
    # -----------------------------
    lidar.stop_motor()
    lidar.stop()
    time.sleep(1)
    lidar.disconnect()

    # Turn off interactive mode and show the final plot
    plt.ioff()
    plt.show()
