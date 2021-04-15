import sys
import time
from datetime import datetime
from datetime import date
import matplotlib.pyplot as plt
from collections import OrderedDict
import deepdish as dd
import os
import numpy as np
from matplotlib import ticker
import zmq

os.chdir("/home/labuser/Desktop/googledrive/code/Samarium_control/Widgets/Objects")
import LJProbeSolenoidControl as lj_probe
os.chdir("/home/labuser/Desktop/googledrive/code/Samarium_control/Widgets/python_gui_v2")

## Stream Grabber
class StreamGrabber():
    def __init__(self,port=5563,topic='topic',ip_addr = 'localhost'):
        self.context = zmq.Context()
        self.port = port
        self.topic = topic
        self.ip_addr = ip_addr
        self.makeConnection()

    def makeConnection(self):
        # Socket to talk to server
        self.socket = self.context.socket(zmq.SUB)
        connect_string = "tcp://%s:%s" % (self.ip_addr,self.port)
        self.socket.connect(connect_string)
        self.socket.setsockopt(zmq.SUBSCRIBE, self.topic.encode('utf-8'))

    def grabData(self):
        try:
            string = self.socket.recv(flags=zmq.NOBLOCK)
        except zmq.ZMQError as e: #nothing received
            string=''
            #print(e)
        return string

    def streamData(self):
        while(True):
            time.sleep(1)
            try:
                print(self.grabData())
            except KeyboardInterrupt:
                break

## Temperature Resistance Conversion (fron TH10K Thermistor datasheet)
def TempResistConv(R):
    if R < 692.6 and R >= 32.77:
        a = 3.3570420e-3
        b = 2.5214848e-4
        c = 3.3743283e-6
        d = -6.4957311e-8
    elif R < 32.77 and R >= 3.599:
        a = 3.3540170e-3
        b = 2.5617244e-4
        c = 2.1400943e-6
        d = -7.2405219e-8
    elif R < 3.599 and R >= 0.6816:
        a = 3.3530481e-3
        b = 2.5420230e-4
        c = 1.1431163e-6
        d = -6.9383563e-8
    elif R < 0.6816 and R >= 0.187:
        a = 3.3536166e-3
        b = 2.5377200e-4
        c = 8.5433271e-7
        d = -8.7912262e-8
    R0 = 10 # kOhm
    T = 1/(a+b*np.log(R/R0)+2*c*np.log(R/R0)+3*d*np.log(R/R0))-273.15 # in Celsuis
    return T

def ResistTempConv(T):
    if T < -1 and T >= -50:
        a = -1.6443767e+1
        b = 6.1080608e+3
        c = -4.4141671e+5
        d = 2.4159818e+7
    elif T < 49 and T >= -1:
        a = -1.5470381e+1
        b = 5.6022839e+3
        c = -3.7886070e+5
        d = 2.4971623e+7
    elif T < 99 and T >= 49:
        a = -1.4807463e+1
        b = 5.1550854e+3
        c = -2.9717659e+5
        d = 2.2904187e+7
    elif T < 150 and T >= 99:
        a = -1.4862658e+1
        b = 5.2676519e+3
        c = -3.5374848e+5
        d = 3.1207901e+7
    R0 = 10      # kOhm
    T = T+273.15 # Kelvin
    R = R0*np.exp(a+b/T+c/T**2+d/T**3)
    return R

## Get Frequency, Current, Temperature, and Time
stream = StreamGrabber(port=5563,topic='wavemeter')

def parse(stream):
    data_string = stream.grabData()
    if isinstance(data_string,bytes):
        data_string = (data_string.decode()).split(',')
        #print(data_string)
        count = 0
        while len(data_string) < 3 and count < 10:
            time.sleep(0.1)
            data_string = stream.grabData()
            data_string = (data_string.decode()).split(',')
            count = count + 1
            #print(data_string)
        timestamp = str(datetime.fromtimestamp(float(data_string[1])))
        channel = int(data_string[2])
        frequency = float(data_string[3])
        #if channel in [3,4,7]: #print(data_string)
        return timestamp,channel,frequency
    else:
        return 0,0,0

def find_freq():
    timestamp,chan,freq = parse(stream)

    start = time.time()
    t_elapsed = 0
    while chan != 4 and t_elapsed < 60:
        t_elapsed = time.time() - start
        timestamp,chan,freq = parse(stream)
    return freq

def find_current():
    curr = reference_current.get_current()
    return curr

def find_temp():
    temp = TempResistConv(find_resist())
    return temp

def find_resist():
    resist = reference_current.get_temperature()
    return resist

def find_time():
    timestamp,chan,freq = parse(stream)
    return timestamp

## Change Current and Temperature
def change_current(c):
    curr = c + 0.4
    reference_current.move_current(curr)        # change current
    print("Current is changed to " + str(c) + ".")

def change_temp(temp): # input voltage to be converted into resistance
    # Note that the input voltage is with conversion factor: R = V*2
    # Note that the is an offset of around 0.13-0.15kOhm between the computer reading and the temp controller display, does not affect the code
    resist = ResistTempConv(temp)
    reference_current.set_temperature(resist) # input for temperature
    print("Want to change the temperature to " + str(temp) + ".")

## Data Analysis Functions
def grab_data(name):
    path = "/home/labuser/googledrive/code/Samarium_analysis/Data/Laser_Mode/"
    freq_path = path + name + " Data_for_freq_arr.txt"
    temp_path = path + name + " Data_for_temp_arr.txt"
    curr_path = path + name + " Data_for_curr_arr.txt"
    temp_arr = np.loadtxt(temp_path)
    curr_arr = np.loadtxt(curr_path)
    freq_arr = np.loadtxt(freq_path)
    return temp_arr, curr_arr, freq_arr

def find_optimal_setpoint(name, lower, higher):
    '''
    name is string of file name for specific scan
    lower and higher indicate the range of frequency desired
    '''
    temp_arr, curr_arr, freq_arr = grab_data(name)
    freq_a = np.reshape(freq_arr, (len(temp_arr), len(curr_arr)))

    usable_points = []
    usable_index = []
    for i in range(len(freq_a)):
        for j in range(len(freq_a[i])):
            if freq_a[i][j] < higher and freq_a[i][j] > lower:
                usable_points.append([temp_arr[i], curr_arr[j]])
                usable_index.append([i, j])
    return usable_points, usable_index

def find_freq_based_on_curr_temp(name, temp, curr):
    temp_arr, curr_arr, freq_arr = grab_data(name)
    freq_a = np.reshape(freq_arr, (len(temp_arr), len(curr_arr)))

    i_index = []
    for i in range(len(temp_arr)):
        if round(temp_arr[i], 1) == 17.3:
            i_index.append([i])
    j_index = []
    for j in range(len(curr_arr)):
        if round(curr_arr[j]) == 106:
            j_index.append([j])

    freqs = []
    for k in range(len(i_index)):
        for h in range(len(j_index)):
            freqs.append(freq_a[k][h])
    return freqs

## Simple Contour Plot
def contour_plot(name, freq, ran):
    temp_arr, curr_arr, freq_arr = grab_data(name)
    freq_a = np.reshape(freq_arr, (len(temp_arr), len(curr_arr)))

    fig, ax = plt.subplots()
    # color bar
    plt.contourf(curr_arr, temp_arr, freq_a, levels = np.linspace(freq-50, freq+50, 50)) #interpolation = 'gaussian')
    plt.colorbar()

    # find in range values
    in_range_freq, in_range_index = find_optimal_setpoint(name, freq-ran, freq+ran)
    use_curr = []
    use_temp = []
    for i in range(len(in_range_index)):
        use_curr.append(in_range_freq[i][1])
        use_temp.append(in_range_freq[i][0])
    # indicate in range values on plot
    sc = plt.scatter(use_curr, use_temp, color = 'red', label = str(freq) + '+/- ' + str(ran))

    plt.legend()
    ax.set_title('Laser Geography Map')
    ax.set_xlabel('Current (mA)')
    ax.set_ylabel('Temperature (C)')
    plt.show()

## RUN THIS SHELL Find laser mode
# Frequeny, Current and Temperature array
curr_step = 0.1
temp_step = 0.1
start_curr = 113
end_curr = 100
start_temp = find_temp()
end_temp = start_temp + 10

def find_laser_modes(start_temp, end_temp, start_curr, end_curr, temp_step, curr_step):
    freq_arr = []
    temp_arr = []
    curr_arr = np.linspace(start_curr, end_curr, np.abs(start_curr-end_curr)/curr_step)
    temp_arr = np.linspace(start_temp, end_temp, np.abs(start_temp-end_temp)/temp_step)

    # fill in frequency
    start_time = time.time()
    count_temp = 0
    original_temp = []
    for j in range(len(temp_arr)):
        change_temp(temp_arr[j])
        time.sleep(35)
        if count_temp % 2 == 0:
            for i in range(len(curr_arr)):
                change_current(curr_arr[i])
                time.sleep(0.2)
                stream = StreamGrabber(port=5563,topic='wavemeter')
                time.sleep(0.01)
                count = 0
                while count < 5:
                    freq = find_freq()
                    if freq > 0:
                        break;
                    count += 1
                freq_arr.append(freq)
        else:
            for i in range(len(curr_arr)):
                change_current(curr_arr[len(curr_arr)-i-1])
                time.sleep(0.2)
                stream = StreamGrabber(port=5563,topic='wavemeter')
                time.sleep(0.01)
                count = 0
                while count < 5:
                    freq = find_freq()
                    if freq > 0:
                        break;
                    count += 1
                freq_arr.append(freq)

        count_temp += 1
        ori_temp = find_temp()
        original_temp.append(ori_temp)

    end_time = time.time()
    time_taken = end_time-start_time
    return curr_arr, temp_arr, freq_arr, original_temp, time_taken

curr_arr, temp_arr, freq_arr, original_temp, time_taken = find_laser_modes(start_temp, end_temp, start_curr, end_curr, temp_step, curr_step)

## RUN THIS SHELL save data
def save_data(date, range):
    path = '/home/labuser/googledrive/code/Samarium_analysis'
    folder_name = "/Data/Laser_Mode"
    full_path = path + folder_name + "/"
    current_time = datetime.now().time()

    np.savetxt(full_path + date + " " + range + str(current_time) + ' Data_for_temp_arr.txt', temp_arr)
    np.savetxt(full_path + date + " " + range + str(current_time) + ' Data_for_curr_arr.txt', curr_arr)
    np.savetxt(full_path + date + " " + range + str(current_time) + ' Data_for_freq_arr.txt', freq_arr)
    np.savetxt(full_path + date + " " + range + str(current_time) + ' Original_temp.txt', original_temp)

save_data("03-27", "10-20")

## Hover Graph
name = '03-11 10-2017:02:24.261536' #'03-27 10-2010:05:44.982752' #'03-04 10-2017:07:08.573611'
freq = 438039.5
ran = 1

def update_annot(ind):
    pos = sc.get_offsets() [ind["ind"][0]]
    annot.xy = pos
    print(pos)
    pairs, not_used = find_optimal_setpoint(name, freq-ran, freq+ran)
    text = "("+str(round(pos[0], 2))+", "+str(round(pos[1], 2))+")"
    annot.set_text(text)
    annot.get_bbox_patch().set_alpha(0.4)

def hover(event):
    vis = annot.get_visible()
    if event.inaxes == ax:
        cont, ind = sc.contains(event)
        #print('cont' + str(cont) + "ind" + str(ind))
        if cont:
            update_annot(ind)
            annot.set_visible(True)
            fig.canvas.draw_idle()
        else:
            if vis:
                annot.set_visible(False)
                fig.canvas.draw_idle()

# configure basic plot settings
fig, ax = plt.subplots()
annot = ax.annotate('', xy = (0,0), xytext = (20,20), textcoords = "figure pixels", bbox = dict (boxstyle = "round", fc = 'w'), arrowprops = dict(arrowstyle = "->"))

#def contour_plot(name, freq, ran):
temp_arr, curr_arr, freq_arr = grab_data(name)
freq_a = np.reshape(freq_arr, (len(temp_arr), len(curr_arr)))

# color bar
plt.contourf(curr_arr, temp_arr, freq_a, levels = np.linspace(freq-50, freq+50, 50)) #interpolation = 'gaussian')
plt.colorbar()

# find in range values
in_range_freq, in_range_index = find_optimal_setpoint(name, freq-ran, freq+ran)
use_curr = []
use_temp = []
for i in range(len(in_range_index)):
    use_curr.append(in_range_freq[i][1])
    use_temp.append(in_range_freq[i][0])
# indicate in range values on plot
sc = plt.scatter(use_curr, use_temp, color = 'red', label = str(freq) + '+/- ' + str(ran))

plt.legend()
ax.set_title('Laser Geography Map')
ax.set_xlabel('Current (mA)')
ax.set_ylabel('Temperature (C)')
fig.canvas.mpl_connect("motion_notify_event", hover)
plt.show()

## Test Temperature change time
temp = find_temp()
change_temp(temp+0.1)

start_time = time.time()
temp_arr = []
end_time = time.time()
time_taken = end_time-start_time
while time_taken < 150:
    temp_now = find_temp()
    temp_arr.append(temp_now)
    end_time = time.time()
    time_taken = end_time-start_time
    plt.plot(float(time_taken), temp_now, '.', color = 'b')
    plt.pause(1e-9)
    #plt.plot(float(time_taken), temp-0.08, '.', color = 'b')
temp = find_temp()
print('Current temperature is ' + str(temp) +".")

## Test Streamgrabber
start_time = time.time()
stream = StreamGrabber(port=5563,topic='wavemeter')
t = find_time()
print(t)

start_time = time.time()
stream = StreamGrabber(port=5563,topic='wavemeter')
freq = find_freq()
end_time = time.time()
time_taken = end_time-start_time

print(freq)
print(time_taken)

## Test Response time
lower_point = 16
upper_point = 16.35
step_size = 0.035
num_steps = int((upper_point-lower_point)/step_size)

change_temp(lower_point)
time.sleep(30)
diff_decrease = []
diff_increase = []

for i in range(num_steps):
    # decreasing from set value
    new_temp = upper_point - step_size*i
    change_temp(new_temp)
    time.sleep(30)
    current_temp = find_temp()
    diff = new_temp - current_temp
    diff_decrease.append([step_size*i, diff])

    change_temp(upper_point)
    time.sleep(30)
    current_temp = find_temp()
    diff = upper_point - current_temp
    diff_increase.append([step_size*i, diff])

print(diff_decrease)
print(diff_increase)

temperature_array = np.linspace(upper_point, lower_point, num_steps)
num = np.linspace(0, 2, 20)
lower = np.linspace(16.5, 16.5, 20)
upper = np.linspace(18.5, 18.5, 20)
plt.plot(temperature_array)
temp_de = []
temp_in = []
for i in range(len(diff_increase)):
    new_temp = upper_point - step_size*i
    temp_in.append(-diff_increase[i][1] + upper_point)
    temp_de.append(-diff_decrease[i][1] + new_temp)
plt.plot(temp_in, '.', color = "blue")
plt.plot(temp_de, '.', color = 'red')

plt.legend(['set_point', 'increased to', 'decreased to'])
plt.plot(lower)
plt.plot(upper)
plt.show()

# A difference of greater than 1.1 degree decrease is not handled well by the temp controller. It will no longer have a constant temprature offset.
