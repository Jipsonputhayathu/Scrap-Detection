import time
import threading
import ctypes as ct
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from opcua import Client
import pyllt as llt
import sys
from scipy.signal import savgol_filter, medfilt
from scipy.stats import zscore
from scipy.ndimage import gaussian_filter1d

start_time = time.time()

# Connect to OPC UA server to get real-time motor position
opc_client = Client("opc.tcp://192.168.0.2:4840")  # Replace with your OPC UA server details
opc_client.connect()
print("Connected to OPC UA Server!")

# NodeId for motor position
position_node = opc_client.get_node("ns=6;s=::Handling:instJS_MC_ReadActualPosition.Position")

profile_count = 0  # Global profile count for terminal display

def get_real_motor_position():
    """Retrieve real-time motor position in degrees."""
    try:
        current_position = position_node.get_value()
        return (current_position / 120000) * 360  # Convert position to degrees
    except Exception as e:
        print(f"Error reading motor position from OPC UA: {e}")
        return 0  # Return 0 if there's an error

def update_terminal_display():
    """Continuously updates the terminal with the motor position and profile count."""
    global profile_count
    while True:
        motor_position = get_real_motor_position()
        sys.stdout.write(f"\rMotor Position: {motor_position:.2f}° | Profiles Processed: {profile_count}")
        sys.stdout.flush()
        time.sleep(1)

# Start the terminal display in a separate thread
terminal_thread = threading.Thread(target=update_terminal_display, daemon=True)
terminal_thread.start()

def filter_laser_data(x, z, filter_value=0.0):
    mask = (x != filter_value) & (z != filter_value)
    return x[mask], z[mask]

def remove_outliers_with_zscore(x, z, threshold=3):
    """Remove outliers based on Z-score if data is sufficiently varied."""
    if len(z) > 1 and np.std(z) > 0:  # Ensure there is some variation
        z_scores = np.abs(zscore(z))  # Calculate Z-scores
        mask = z_scores < threshold   # Filter out points where Z-score > threshold
        return x[mask], z[mask]
    else:
        print("Warning: Insufficient variance in data for Z-score. Returning original data.")
        return x, z  # Return original data if there's no variance


def smooth_data_with_savgol(x, z, window_size=5, poly_order=3):
    """Apply Savitzky-Golay filter for smoothing, if data is sufficient."""
    if len(z) >= window_size:  # Ensure the data length meets the window size
        z_smoothed = savgol_filter(z, window_size, poly_order)
    else:
        print("Warning: Insufficient data for Savitzky-Golay filter. Returning original data.")
        z_smoothed = z  # Return the original data if not enough points
    return x, z_smoothed


def smooth_data_with_median(x, z, kernel_size=3):
    z_smoothed = medfilt(z, kernel_size=kernel_size)
    return x, z_smoothed

def smooth_data_with_gaussian(x, z, sigma=1.0):
    z_smoothed = gaussian_filter1d(z, sigma=sigma)
    return x, z_smoothed

def profile_callback(data, size, user_data):
    global profile_buffer, profile_buffer1, received, received1
    if user_data == 1:
        ct.memmove(profile_buffer, data, size)
        received = True
    elif user_data == 2:
        ct.memmove(profile_buffer1, data, size)
        received1 = True

    if received and received1:
        event.set()
        received1 = False
        received = False

def transform_coordinates_from_center(x, z, angle_deg, radius):
    angle_rad = np.deg2rad(angle_deg)
    x_shifted = x - x[0]
    z_shifted = z - z[0]
    x_rot = x_shifted * np.cos(angle_rad) - z_shifted * np.sin(angle_rad)
    z_rot = x_shifted * np.sin(angle_rad) + z_shifted * np.cos(angle_rad)
    x_translated = x_rot + radius * np.cos(angle_rad)
    z_translated = z_rot + radius * np.sin(angle_rad)
    return x_translated, z_translated

# Initialize
received = False
received1 = False
exposure_time_units = 12
idle_time_units = 450
start_data = 4
data_width = 4
scanner_type = ct.c_int(0)
scanner_type1 = ct.c_int(0)

available_resolutions = (ct.c_uint * 4)()
get_profile_cb = llt.buffer_cb_func(profile_callback)
event = threading.Event()

null_ptr_short = ct.POINTER(ct.c_ushort)()
null_ptr_int = ct.POINTER(ct.c_uint)()

hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
hLLT1 = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)

def setup_device(device, ip_address):
    ret = llt.set_device_interface(device, ip_address, 0)
    if ret < 1:
        raise ValueError("Error setting device interface: " + str(ret))
    ret = llt.connect(device)
    if ret < 1:
        raise ConnectionError("Error connect: " + str(ret))
    return device

hLLT = setup_device(hLLT, 3232235524)
hLLT1 = setup_device(hLLT1, 3232235527)

def set_resolution(device, available_resolutions):
    ret = llt.get_resolutions(device, available_resolutions, len(available_resolutions))
    if ret < 1:
        raise ValueError("Error getting resolutions : " + str(ret))
    resolution = available_resolutions[0]
    ret = llt.set_resolution(device, resolution)
    if ret < 1:
        raise ValueError("Error setting resolution : " + str(ret))
    return resolution

resolution = set_resolution(hLLT, available_resolutions)
resolution1 = set_resolution(hLLT1, available_resolutions)

def set_laser_params(device):
    ret = llt.set_feature(device, llt.FEATURE_FUNCTION_EXPOSURE_TIME, exposure_time_units)
    if ret < 1:
        raise ValueError("Error setting exposure time: " + str(ret))
    ret = llt.set_feature(device, llt.FEATURE_FUNCTION_IDLE_TIME, idle_time_units)
    if ret < 1:
        raise ValueError("Error setting idle time: " + str(ret))

set_laser_params(hLLT)
set_laser_params(hLLT1)

profile_buffer = (ct.c_ubyte * (resolution * data_width))()
profile_buffer1 = (ct.c_ubyte * (resolution1 * data_width))()

x = np.empty(resolution, dtype=float)
z = np.empty(resolution, dtype=float)
x_p = x.ctypes.data_as(ct.POINTER(ct.c_double))
z_p = z.ctypes.data_as(ct.POINTER(ct.c_double))

x1 = np.empty(resolution1, dtype=float)
z1 = np.empty(resolution1, dtype=float)
x_p1 = x1.ctypes.data_as(ct.POINTER(ct.c_double))
z_p1 = z1.ctypes.data_as(ct.POINTER(ct.c_double))

partial_profile_struct = llt.TPartialProfile(0, start_data, resolution, data_width)
partial_profile_struct1 = llt.TPartialProfile(0, start_data, resolution1, data_width)

def configure_device(device, profile_struct):
    ret = llt.set_profile_config(device, llt.TProfileConfig.PARTIAL_PROFILE)
    if ret < 1:
        raise ValueError("Error setting profile config: " + str(ret))
    ret = llt.set_feature(device, llt.FEATURE_FUNCTION_TRIGGER, llt.TRIG_INTERNAL)
    if ret < 1:
        raise ValueError("Error setting trigger: " + str(ret))
    ret = llt.set_partial_profile(device, ct.byref(profile_struct))
    if ret < 1:
        raise ValueError("Error setting partial profile: " + str(ret))

configure_device(hLLT, partial_profile_struct)
configure_device(hLLT1, partial_profile_struct1)

def register_callback(device, user_data):
    ret = llt.register_callback(device, llt.TCallbackType.C_DECL, get_profile_cb, user_data)
    if ret < 1:
        raise ValueError("Error setting callback: " + str(ret))

register_callback(hLLT, 1)
register_callback(hLLT1, 2)

def start_transfer():
    ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
    if ret < 1:
        raise ValueError("Error starting transfer profiles: " + str(ret))
    ret = llt.transfer_profiles(hLLT1, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
    if ret < 1:
        raise ValueError("Error starting transfer profiles: " + str(ret))

print("---Press Enter to start measurement and CTRL-C to stop measurement!---")
var = input("")

if var == "":
    start_transfer()
    print("Measurement of both sensors started!")
else:
    print("Please press Enter to start the measurement! Start the program again!")
    sys.exit(0)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))

ax1.set_title("Top Laser Profile (Sensor 1)")
ax1.set_xlabel("X Coordinate (mm)")
ax1.set_ylabel("Z Coordinate (mm)")
ax1.set_aspect('equal')
ax1.grid(True)

ax2.set_title("Bottom Laser Profile (Sensor 2)")
ax2.set_xlabel("X Coordinate (mm)")
ax2.set_ylabel("Z Coordinate (mm)")
ax2.set_aspect('equal')
ax2.grid(True)

# Add a timer text in the first subplot
timer_text = ax1.text(0.02, 0.95, '', transform=ax1.transAxes, fontsize=12, color="red")


def update(data):
    (x_transformed, z_transformed), (x_transformed1, z_transformed1) = data
    ax1.plot(x_transformed, z_transformed, '.', markersize=1)
    ax2.plot(x_transformed1, z_transformed1, '.', markersize=1)
    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()

    # Calculate the elapsed time
    elapsed_time = time.time() - start_time
    timer_text.set_text(f"Elapsed Time: {elapsed_time:.2f} s")

# Global variables for storing full rotation data
full_rotation_top_x = []
full_rotation_top_z = []
full_rotation_bottom_x = []
full_rotation_bottom_z = []

def reset_full_rotation_data():
    global full_rotation_top_x, full_rotation_top_z, full_rotation_bottom_x, full_rotation_bottom_z
    full_rotation_top_x = []
    full_rotation_top_z = []
    full_rotation_bottom_x = []
    full_rotation_bottom_z = []

def store_profile_data(x_top, z_top, x_bottom, z_bottom):
    """Store profile data for each sensor."""
    global full_rotation_top_x, full_rotation_top_z, full_rotation_bottom_x, full_rotation_bottom_z
    full_rotation_top_x.append(x_top)
    full_rotation_top_z.append(z_top)
    full_rotation_bottom_x.append(x_bottom)
    full_rotation_bottom_z.append(z_bottom)

def analyze_full_rotation():
    """Analyze the stored data after a full rotation."""
    # Convert lists to numpy arrays for analysis
    x_top = np.concatenate(full_rotation_top_x)
    z_top = np.concatenate(full_rotation_top_z)
    x_bottom = np.concatenate(full_rotation_bottom_x)
    z_bottom = np.concatenate(full_rotation_bottom_z)
    
    # Example analyses
    top_mean_radius = np.mean(np.sqrt(x_top**2 + z_top**2))
    top_std_radius = np.std(np.sqrt(x_top**2 + z_top**2))
    
    bottom_mean_radius = np.mean(np.sqrt(x_bottom**2 + z_bottom**2))
    bottom_std_radius = np.std(np.sqrt(x_bottom**2 + z_bottom**2))
    
    print("\nTop Sensor Analysis:")
    print(f"Mean Radius: {top_mean_radius:.2f} mm, Std Dev: {top_std_radius:.2f} mm")
    
    print("\nBottom Sensor Analysis:")
    print(f"Mean Radius: {bottom_mean_radius:.2f} mm, Std Dev: {bottom_std_radius:.2f} mm")
    
    # Additional analyses can be added here, such as symmetry, deviations, or feature detection.
    
    reset_full_rotation_data()  # Clear data for the next rotation

# Update `data_gen()` to check for a full rotation and analyze
def data_gen():
    global profile_count
    circle_radius = 25
    last_motor_position = 0

    while True:
        event.wait()
        motor_position_degrees = get_real_motor_position()

        # Store motor position to detect full rotation
        if motor_position_degrees < last_motor_position:
            analyze_full_rotation()
        
        last_motor_position = motor_position_degrees

        # Convert top laser data
        fret = llt.convert_part_profile_2_values(hLLT, profile_buffer, ct.byref(partial_profile_struct), scanner_type, 0, 1,
                                                 null_ptr_short, null_ptr_short, null_ptr_short, x_p, z_p, null_ptr_int, null_ptr_int)
        if fret & llt.CONVERT_X == 0 or fret & llt.CONVERT_Z == 0:
            print("Error converting data for Sensor 1")
            continue

        filtered_x, filtered_z = filter_laser_data(x, z)
        x_transformed, z_transformed = transform_coordinates_from_center(filtered_x, filtered_z, motor_position_degrees, circle_radius)

        # Convert bottom laser data
        fret1 = llt.convert_part_profile_2_values(hLLT1, profile_buffer1, ct.byref(partial_profile_struct1), scanner_type1, 0, 1,
                                                  null_ptr_short, null_ptr_short, null_ptr_short, x_p1, z_p1, null_ptr_int, null_ptr_int)
        if fret1 & llt.CONVERT_X == 0 or fret1 & llt.CONVERT_Z == 0:
            print("Error converting data for Sensor 2")
            continue

        filtered_x1, filtered_z1 = filter_laser_data(x1, z1)
        x_transformed1, z_transformed1 = transform_coordinates_from_center(filtered_x1, filtered_z1, motor_position_degrees, circle_radius)

        # Ensure both top and bottom profiles are ready before storing
        if x_transformed is not None and z_transformed is not None and x_transformed1 is not None and z_transformed1 is not None:
            store_profile_data(x_transformed, z_transformed, x_transformed1, z_transformed1)

        profile_count += 1
        event.clear()

        # Yield the transformed profiles for top and bottom laser
        yield (x_transformed, z_transformed), (x_transformed1, z_transformed1)

ani = animation.FuncAnimation(fig, update, frames=data_gen, interval=10, blit=False, cache_frame_data=False)

plt.show()

def cleanup():
    ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
    if ret < 1:
        print("Error stopping transfer profiles for Sensor 1")
    ret = llt.transfer_profiles(hLLT1, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
    if ret < 1:
        print("Error stopping transfer profiles for Sensor 2")
    llt.disconnect(hLLT)
    llt.disconnect(hLLT1)
    llt.del_device(hLLT)
    llt.del_device(hLLT1)

    # Disconnect from OPC UA server
    opc_client.disconnect()
    print("Cleanup completed, sensors disconnected and OPC UA server disconnected.")

cleanup()
