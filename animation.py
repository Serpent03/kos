#   Import needed functions. You would need to install matplotlib via pip.
import csv
import matplotlib.pyplot as plt 
from matplotlib.animation import FuncAnimation

# get the necessary data

velocity = []
altitude = []
shipQ = []

with open('fltdata.csv', 'r') as f: # I read the values from a csv file, logged by kOS.
    reader = csv.reader(f)
    for row in reader:
        velocity.append(float(row[0]))
        altitude.append(float(row[1]))
        shipQ.append(float(row[2]))

#print(velocity[0], altitude[0], shipQ[0])   # Just for debug.

# shift the data onto matplotlib

x = []
y = []
z = []

plt.style.use("Solarize_Light2")
fig = plt.figure()
ax = plt.axes(xlim=(-1000,0), ylim=(altitude[-1],altitude[0]))  # set the x-axis and y-axis limit. For now we're just plotting speed/altitude on x/y
line, = ax.plot([], [])

# Axis labels
ax.set_xlabel('Vertical Speed (m/s)')
ax.set_ylabel('Ship Altitude (m)')
ax.set_facecolor("grey")
plt.legend()

def initFrame():
    line.set_data([], [])   # make sure the initial value is not defined so it doesn't bug in a repeating loop :)
    return line

def updateData(i):
    x.append(velocity[i])
    y.append(altitude[i])
    #z.append(shipQ[i])  # I haven't made the effort to have more than 1 graph line as of yet, so sorry about that :P

    line.set_data(x, y) # now for each frame, this sets the data onto the graph, parsed by the .append function from that specific index of the vel/alt list.
    line.set_color("white")

    return line

# as you can see, frames is what gets passed onto updateData(i), which is equal to the length of the list. All the lists have the same length. This ensures that
# when we refer to the data in updateData(i), the value of i is always the specific index at for all lists, so it is continuous :)

anim = FuncAnimation(fig, updateData, init_func=initFrame, frames=len(velocity), interval=5, repeat=False)
plt.show()