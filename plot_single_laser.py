import time
import threading
import ctypes as ct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import pyllt as llt

def profile_callback(data, size, user_data):
    global profile_buffer
    if user_data == 1:
        ct.memmove(profile_buffer, data, size)
        event.set()

# Parametrize transmission
start_data = 0
data_width = 8
scanner_type = ct.c_int(0)

# Init profile buffer and timestamp info
timestamp = (ct.c_ubyte * 16)()
available_resolutions = (ct.c_uint * 4)()
available_interfaces = (ct.c_uint * 6)()
lost_profiles = ct.c_int()
shutter_opened = ct.c_double(0.0)
shutter_closed = ct.c_double(0.0)
profile_count = ct.c_uint(0)

# Callback function
get_profile_cb = llt.buffer_cb_func(profile_callback)
event = threading.Event()

# Null pointer if data not necessary
null_ptr_short = ct.POINTER(ct.c_ushort)()
null_ptr_int = ct.POINTER(ct.c_uint)()

# Create instance and set IP address
hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)

# Get available interfaces
ret = llt.get_device_interfaces_fast(hLLT, available_interfaces, len(available_interfaces))
if ret < 1:
    raise ValueError("Error getting interfaces : " + str(ret))

ret = llt.set_device_interface(hLLT, available_interfaces[0], 0)
if ret < 1:
    raise ValueError("Error setting device interface: " + str(ret))

# Connect
ret = llt.connect(hLLT)
if ret < 1:
    raise ConnectionError("Error connect: " + str(ret))

# Get available resolutions
ret = llt.get_resolutions(hLLT, available_resolutions, len(available_resolutions))
if ret < 1:
    raise ValueError("Error getting resolutions : " + str(ret))

# Set max. resolution
resolution = available_resolutions[0]
ret = llt.set_resolution(hLLT, resolution)
if ret < 1:
    raise ValueError("Error getting resolutions : " + str(ret))

# Declare measuring data arrays
profile_buffer = (ct.c_ubyte*(resolution * data_width))()
x = np.empty(resolution, dtype=float)  # (ct.c_double * resolution)()
z = np.empty(resolution, dtype=float)  # (ct.c_double * resolution)()
x_p = x.ctypes.data_as(ct.POINTER(ct.c_double))
z_p = z.ctypes.data_as(ct.POINTER(ct.c_double))
intensities = (ct.c_ushort * resolution)()

# Partial profile struct
partial_profile_struct = llt.TPartialProfile(0, start_data, resolution, data_width)

# Scanner type
ret = llt.get_llt_type(hLLT, ct.byref(scanner_type))
if ret < 1:
    raise ValueError("Error scanner type: " + str(ret))

# Scanner type
ret = llt.set_resolution(hLLT, resolution)
if ret < 1:
    raise ValueError("Error setting resolution: " + str(ret))

# Set partial profile as profile config
ret = llt.set_profile_config(hLLT, llt.TProfileConfig.PARTIAL_PROFILE)
if ret < 1:
    raise ValueError("Error setting profile config: " + str(ret))

# Set trigger
ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_TRIGGER, llt.TRIG_INTERNAL)
if ret < 1:
    raise ValueError("Error setting trigger: " + str(ret))

# Set exposure time
ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_EXPOSURE_TIME, 100)
if ret < 1:
    raise ValueError("Error setting exposure time: " + str(ret))

# Set idle time
ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_IDLE_TIME, 3900)
if ret < 1:
    raise ValueError("Error idle time: " + str(ret))

# Set partial profile
ret = llt.set_partial_profile(hLLT, ct.byref(partial_profile_struct))
if ret < 1:
    raise ValueError("Error setting partial profile: " + str(ret))

# Register Callback
ret = llt.register_callback(hLLT, llt.TCallbackType.C_DECL, get_profile_cb, 1)
if ret < 1:
    raise ValueError("Error setting callback: " + str(ret))

# Start transfer
ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
if ret < 1:
    raise ValueError("Error starting transfer profiles: " + str(ret))

# Warm-up time
time.sleep(0.1)

# List to store all captured data
all_data = []

def data_gen():
    while True:
        event.wait()
        fret = llt.convert_part_profile_2_values(hLLT, profile_buffer, ct.byref(partial_profile_struct), scanner_type, 0, 1,
                                                 null_ptr_short, null_ptr_short, null_ptr_short, x_p, z_p, null_ptr_int, null_ptr_int)
        if fret & llt.CONVERT_X == 0 or fret & llt.CONVERT_Z == 0:
            raise ValueError("Error converting data: " + str(ret))

        for i in range(16):
            timestamp[i] = profile_buffer[resolution * data_width - 16 + i]

        llt.timestamp_2_time_and_count(timestamp, ct.byref(shutter_opened), ct.byref(shutter_closed), ct.byref(profile_count))
        event.clear()

        # Store the data
        all_data.append((x.copy(), z.copy()))

        yield x, z

# Capture data for a specific duration or number of profiles
capture_duration = 31  # seconds
start_time = time.time()

fig, ax = plt.subplots()
line, = ax.plot([], [], ".b", lw=2)
ax.grid()
ax.set_xlim(-60, 60)
ax.set_ylim(25, 350)
line.set_data(x, z)

def update(data):
    ux, uz = data
    line.set_data(ux, uz)
    return line,

ani = animation.FuncAnimation(fig, update, frames=data_gen, interval=40, blit=True, cache_frame_data=False)

plt.show()

# Stop the data capture
ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
if ret < 1:
    raise ValueError("Error stopping transfer profiles: " + str(ret))

# Disconnect
ret = llt.disconnect(hLLT)
if ret < 1:
    raise ConnectionAbortedError("Error while disconnect: " + str(ret))

# Delete
ret = llt.del_device(hLLT)
if ret < 1:
    raise ConnectionAbortedError("Error while delete: " + str(ret))

# Create a 3D plot with the collected data
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Convert all_data to a numpy array for easier manipulation
all_data = np.array(all_data)
num_profiles = len(all_data)

# Plot each profile with a different color
for i in range(num_profiles):
    xs, zs = all_data[i]
    ys = np.full_like(xs, i)  # Assign y based on the profile number
    ax.plot(xs, ys, zs, color=plt.cm.viridis(i / num_profiles))

# Set plot labels
ax.set_xlabel('X')
ax.set_ylabel('Profile Number')
ax.set_zlabel('Z')
ax.set_title('3D Profile of Rotating Workpiece')

plt.show()
