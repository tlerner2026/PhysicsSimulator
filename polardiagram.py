import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# parse
with open('test.pol', 'r') as file:
    lines = file.read().strip().split('\n')
    
wind_speeds = [float(x) for x in lines[0].split(';')[1:]]

angles = []
speeds_by_wind = [[] for _ in wind_speeds]
for line in lines[1:-1]:
    values = line.split(';')
    angles.append(float(values[0]))
    for i, speed in enumerate(values[1:]):
        speeds_by_wind[i].append(float(speed))
fig = plt.figure(figsize=(10, 9))
ax = fig.add_subplot(111, projection='polar')

# plot data
angles_rad = np.radians(angles)
for speeds, wind_speed in zip(speeds_by_wind, wind_speeds):
    ax.plot(angles_rad, speeds, label=f'{wind_speed} kts')
    ax.plot(2*np.pi - angles_rad, speeds)

ax.set_theta_zero_location('N')
ax.set_theta_direction(-1)
ax.set_rticks([]) # idk what the radial ticks are supposed to be, obviously represents boat speed but we dont calculate theoretical spede i thinnk
ax.set_xticks(np.radians(np.arange(0, 360, 15)))
ax.set_xticklabels([f'{x}°' for x in range(0, 360, 15)])

ax.legend(title='True Wind Speed', bbox_to_anchor=(1.15, 0.5), loc='center right')

# add zoom buttons, normal zoom wasnt working idk why
zoom_in_ax = plt.axes([0.7, 0.05, 0.1, 0.075])
zoom_out_ax = plt.axes([0.81, 0.05, 0.1, 0.075])
zoom_in = Button(zoom_in_ax, 'Zoom In')
zoom_out = Button(zoom_out_ax, 'Zoom Out')

def zoom(factor):
    current_limits = ax.get_ylim()
    ax.set_ylim(0, current_limits[1] * factor)
    plt.draw()

zoom_in.on_clicked(lambda event: zoom(0.8))
zoom_out.on_clicked(lambda event: zoom(1.25))

plt.show()