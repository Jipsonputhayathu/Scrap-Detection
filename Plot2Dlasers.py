import time
import threading
import ctypes as ct
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button, TextBox
import pyllt as llt
import sys
import signal

def profile_callback(data, size, user_data):
    global profile_buffer, profile_buffer1, received, received1
    if user_data == 1:
        ct.memmove(profile_buffer, data, size)
        received = True
    if user_data == 2:
        ct.memmove(profile_buffer1, data, size)
        received1 = True

    if received and received1:
        event.set()
        received1 = False
        received = False

def update_exposure_time_and_frequency(event):
    global exposure_time, idle_time, hLLT, hLLT1
    try:
        exposure_time = int(exposure_time_text_box.text)
        profile_frequency = float(profile_frequency_text_box.text)
        idle_time = int((1 / profile_frequency) * 1e6 - exposure_time)

        exposure_code = (((exposure_time % 10) << 12) & 0xF000) + ((exposure_time // 10) & 0xFFF)
        idle_code = (((idle_time % 10) << 12) & 0xF000) + ((idle_time // 10) & 0xFFF)

        llt.set_feature(hLLT, llt.FEATURE_FUNCTION_EXPOSURE_TIME, exposure_code)
        llt.set_feature(hLLT, llt.FEATURE_FUNCTION_IDLE_TIME, idle_code)

        llt.set_feature(hLLT1, llt.FEATURE_FUNCTION_EXPOSURE_TIME, exposure_code)
        llt.set_feature(hLLT1, llt.FEATURE_FUNCTION_IDLE_TIME, idle_code)

        print(f"Updated Exposure Time: {exposure_time} µs, Profile Frequency: {profile_frequency} Hz")
    except ValueError:
        print("Invalid input for exposure time or profile frequency")

# Initialize
run = 0
received = False
received1 = False

# Parametrize partial profile that only the moment 0 column is transmitted
start_data = 4
data_width = 4
scanner_type = ct.c_int(0)
scanner_type1 = ct.c_int(0)

# Init profile buffer and timestamp info
timestamp = (ct.c_ubyte * 16)()
available_resolutions = (ct.c_uint * 4)()
available_interfaces = (ct.c_uint * 6)()
lost_profiles = ct.c_int()
shutter_opened = ct.c_double(0.0)
shutter_closed = ct.c_double(0.0)
profile_count = ct.c_uint(0)

# Init profile buffer and timestamp info for sensor 2
timestamp1 = (ct.c_ubyte * 16)()
available_resolutions1 = (ct.c_uint * 4)()
available_interfaces1 = (ct.c_uint * 6)()
lost_profiles1 = ct.c_int()
shutter_opened1 = ct.c_double(0.0)
shutter_closed1 = ct.c_double(0.0)
profile_count1 = ct.c_uint(0)

# Callback function
get_profile_cb = llt.buffer_cb_func(profile_callback)
event = threading.Event()

# Null pointer if data not necessary
null_ptr_short = ct.POINTER(ct.c_ushort)()
null_ptr_int = ct.POINTER(ct.c_uint)()

# Create instance and set IP address
hLLT = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)
hLLT1 = llt.create_llt_device(llt.TInterfaceType.INTF_TYPE_ETHERNET)

# Get available interfaces
ret = llt.get_device_interfaces_fast(hLLT, available_interfaces, len(available_interfaces))
if ret < 1:
    raise ValueError("Error getting interfaces : " + str(ret))

ret = llt.set_device_interface(hLLT, available_interfaces[0], 0)
if ret < 1:
    raise ValueError("Error setting device interface: " + str(ret))

ret = llt.set_device_interface(hLLT1, available_interfaces[1], 0)
if ret < 1:
    raise ValueError("Error setting device interface: " + str(ret))

# Connect to sensors
ret = llt.connect(hLLT)
if ret < 1:
    raise ConnectionError("Error connect: " + str(ret))

ret = llt.connect(hLLT1)
if ret < 1:
    raise ConnectionError("Error connect: " + str(ret))

devname = ct.create_string_buffer(256)
vename = ct.create_string_buffer(256)
llt.get_device_name(hLLT, devname, len(devname), vename, len(vename))

devname1 = ct.create_string_buffer(256)
vename1 = ct.create_string_buffer(256)
llt.get_device_name(hLLT1, devname1, len(devname1), vename1, len(vename1))

print("Device name of sensor 1: ", devname.value.decode())
print("Device name of sensor 2: ", devname1.value.decode())

# Get available resolutions
ret = llt.get_resolutions(hLLT, available_resolutions, len(available_resolutions))
if ret < 1:
    raise ValueError("Error getting resolutions : " + str(ret))

ret = llt.get_resolutions(hLLT1, available_resolutions1, len(available_resolutions1))
if ret < 1:
    raise ValueError("Error getting resolutions : " + str(ret))

# Set max. resolution
resolution = available_resolutions[0]
ret = llt.set_resolution(hLLT, resolution)
if ret < 1:
    raise ValueError("Error setting resolution : " + str(ret))

resolution1 = available_resolutions1[0]
ret = llt.set_resolution(hLLT1, resolution1)
if ret < 1:
    raise ValueError("Error setting resolution : " + str(ret))

# Declare measuring data arrays
profile_buffer = (ct.c_ubyte * (resolution * data_width))()
x = np.empty(resolution, dtype=float)
z = np.empty(resolution, dtype=float)
x_p = x.ctypes.data_as(ct.POINTER(ct.c_double))
z_p = z.ctypes.data_as(ct.POINTER(ct.c_double))

profile_buffer1 = (ct.c_ubyte * (resolution1 * data_width))()
x1 = np.empty(resolution1, dtype=float)
z1 = np.empty(resolution1, dtype=float)
x_p1 = x1.ctypes.data_as(ct.POINTER(ct.c_double))
z_p1 = z1.ctypes.data_as(ct.POINTER(ct.c_double))

# Partial profile structs
partial_profile_struct = llt.TPartialProfile(0, start_data, resolution, data_width)
partial_profile_struct1 = llt.TPartialProfile(0, start_data, resolution1, data_width)

# Scanner types
ret = llt.get_llt_type(hLLT, ct.byref(scanner_type))
if ret < 1:
    raise ValueError("Error getting scanner type: " + str(ret))

ret = llt.get_llt_type(hLLT1, ct.byref(scanner_type1))
if ret < 1:
    raise ValueError("Error getting scanner type: " + str(ret))

# Set partial profile as profile config
ret = llt.set_profile_config(hLLT, llt.TProfileConfig.PARTIAL_PROFILE)
if ret < 1:
    raise ValueError("Error setting profile config: " + str(ret))

ret = llt.set_profile_config(hLLT1, llt.TProfileConfig.PARTIAL_PROFILE)
if ret < 1:
    raise ValueError("Error setting profile config: " + str(ret))

# Set trigger
ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_TRIGGER, llt.TRIG_INTERNAL)
if ret < 1:
    raise ValueError("Error setting trigger: " + str(ret))

ret = llt.set_feature(hLLT1, llt.FEATURE_FUNCTION_TRIGGER, llt.TRIG_INTERNAL)
if ret < 1:
    raise ValueError("Error setting trigger: " + str(ret))

# Initial exposure time and profile frequency
exposure_time = 231
profile_frequency = 25
idle_time = int((1 / profile_frequency) * 1e6 - exposure_time)

# Set initial exposure time and idle time
exposure_code = (((exposure_time % 10) << 12) & 0xF000) + ((exposure_time // 10) & 0xFFF)
idle_code = (((idle_time % 10) << 12) & 0xF000) + ((idle_time // 10) & 0xFFF)

ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_EXPOSURE_TIME, exposure_code)
if ret < 1:
    raise ValueError("Error setting exposure time: " + str(ret))

ret = llt.set_feature(hLLT, llt.FEATURE_FUNCTION_IDLE_TIME, idle_code)
if ret < 1:
    raise ValueError("Error setting idle time: " + str(ret))

ret = llt.set_feature(hLLT1, llt.FEATURE_FUNCTION_EXPOSURE_TIME, exposure_code)
if ret < 1:
    raise ValueError("Error setting exposure time: " + str(ret))

ret = llt.set_feature(hLLT1, llt.FEATURE_FUNCTION_IDLE_TIME, idle_code)
if ret < 1:
    raise ValueError("Error setting idle time: " + str(ret))

# Set partial profiles
ret = llt.set_partial_profile(hLLT, ct.byref(partial_profile_struct))
if ret < 1:
    raise ValueError("Error setting partial profile: " + str(ret))

ret = llt.set_partial_profile(hLLT1, ct.byref(partial_profile_struct1))
if ret < 1:
    raise ValueError("Error setting partial profile: " + str(ret))

# Register Callback
ret = llt.register_callback(hLLT, llt.TCallbackType.C_DECL, get_profile_cb, 1)
if ret < 1:
    raise ValueError("Error setting callback: " + str(ret))

ret = llt.register_callback(hLLT1, llt.TCallbackType.C_DECL, get_profile_cb, 2)
if ret < 1:
    raise ValueError("Error setting callback: " + str(ret))

measurement_active = False
measurement_stopped = True

# Initialize plot
fig, (ax1, ax2) = plt.subplots(2, 1)
line1, = ax1.plot([], [], ".b", lw=2, label='Sensor 1')
line2, = ax2.plot([], [], ".r", lw=2, label='Sensor 2')
ax1.grid()
ax2.grid()
ax1.set_xlim(-30, 30)
ax1.set_ylim(120, 200)
ax2.set_xlim(-30, 30)
ax2.set_ylim(120, 200)
ax1.set_title('Sensor 1 Data (Blue)')
ax2.set_title('Sensor 2 Data (Red)')
ax1.set_xlabel('X Axis')
ax1.set_ylabel('Z Axis')
ax2.set_xlabel('X Axis')
ax2.set_ylabel('Z Axis')
ax1.legend()
ax2.legend()

# Add GUI elements for updating settings
axcolor = 'lightgoldenrodyellow'
axbox1 = plt.axes([0.1, 0.01, 0.1, 0.04])
axbox2 = plt.axes([0.3, 0.01, 0.1, 0.04])
exposure_time_text_box = TextBox(axbox1, 'Exposure Time (µs)', initial=str(exposure_time))
profile_frequency_text_box = TextBox(axbox2, 'Profile Frequency (Hz)', initial=str(profile_frequency))
update_button_ax = plt.axes([0.5, 0.01, 0.1, 0.04])
update_button = Button(update_button_ax, 'Update Settings')
update_button.on_clicked(update_exposure_time_and_frequency)

def update(data):
    ux, uz, ux1, uz1 = data
    line1.set_data(ux, uz)
    line2.set_data(ux1, uz1)
    ax1.relim()
    ax1.autoscale_view()
    ax2.relim()
    ax2.autoscale_view()
    return line1, line2

def data_gen():
    while measurement_active:
        event.wait()

        fret = llt.convert_part_profile_2_values(hLLT, profile_buffer, ct.byref(partial_profile_struct), scanner_type, 0, 1,
                                                 null_ptr_short, null_ptr_short, null_ptr_short, x_p, z_p, null_ptr_int, null_ptr_int)
        if fret & llt.CONVERT_X == 0 or fret & llt.CONVERT_Z == 0:
            raise ValueError("Error converting data: " + str(fret))

        fret = llt.convert_part_profile_2_values(hLLT1, profile_buffer1, ct.byref(partial_profile_struct1), scanner_type1, 0, 1,
                                                 null_ptr_short, null_ptr_short, null_ptr_short, x_p1, z_p1, null_ptr_int, null_ptr_int)
        if fret & llt.CONVERT_X == 0 or fret & llt.CONVERT_Z == 0:
            raise ValueError("Error converting data: " + str(fret))

        event.clear()
        yield x, z, x1, z1

ani = animation.FuncAnimation(fig, update, frames=data_gen, interval=40, blit=True, cache_frame_data=False)

print("---Press Enter to start measurement and CTRL-C to stop measurement!---")
var = input("")
if var == "":
    ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
    if ret < 1:
        raise ValueError("Error starting transfer profiles: " + str(ret))
    ret = llt.transfer_profiles(hLLT1, llt.TTransferProfileType.NORMAL_TRANSFER, 1)
    if ret < 1:
        raise ValueError("Error starting transfer profiles: " + str(ret))
    print("Measurement of both sensors started!")
    measurement_active = True
    measurement_stopped = False
else:
    print("Please press Enter to start the measurement! Start the program again!")
    sys.exit(0)

plt.show()

# Stop the data capture
ret = llt.transfer_profiles(hLLT, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
if ret < 1:
    raise ValueError("Error stopping transfer profiles: " + str(ret))

ret = llt.transfer_profiles(hLLT1, llt.TTransferProfileType.NORMAL_TRANSFER, 0)
if ret < 1:
    raise ValueError("Error stopping transfer profiles: " + str(ret))

# Disconnect
ret = llt.disconnect(hLLT)
if ret < 1:
    raise ConnectionAbortedError("Error while disconnect: " + str(ret))

ret = llt.disconnect(hLLT1)
if ret < 1:
    raise ConnectionAbortedError("Error while disconnect: " + str(ret))

# Delete
ret = llt.del_device(hLLT)
if ret < 1:
    raise ConnectionAbortedError("Error while delete: " + str(ret))

ret = llt.del_device(hLLT1)
if ret < 1:
    raise ConnectionAbortedError("Error while delete: " + str(ret))
