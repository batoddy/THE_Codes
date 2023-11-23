import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv("AKU1_flight_Data_SD_Card.csv")
x = "Tick"
pkg_min = 1096852
pkg_max = 1113893

# Create the figure and axes
fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(15, 15))

# axes[0, 3].# plot(df['Tick'], df['Thrust'], label='Thrust', color='red')
axes[0].plot(df[x], df["Yaw"], label="Yaw", color="orange")
#axes[0].grid(True)

axes[0].set_xlim(pkg_min, pkg_max)
axes[0].set_xticks(np.arange(pkg_min,pkg_max,500))
axes[0].set_xticklabels(axes[0].get_xticks(),rotation=45, ha='right')

axes[0].set_ylim(-180, 180)
axes[0].set_yticks(np.arange(-180,180,30))

axes[0].legend()

# axes[1, 3].# plot(df['Tick'], df['Thrust'], label='Thrust', color='red')
axes[1].plot(df[x], df["Pitch"], label="Pitch", color="orange")
#axes[1].grid(True)
axes[1].set_xlim(pkg_min, pkg_max)
axes[1].set_xticks(np.arange(pkg_min,pkg_max,500))
axes[1].set_xticklabels(axes[1].get_xticks(),rotation=45, ha='right')

axes[1].set_ylim(-180, 180)
axes[1].set_yticks(np.arange(-180,180,30))

axes[1].legend()

# axes[2, 3].# plot(df['Tick'], df['Thrust'], label='Thrust', color='red')
axes[2].plot(df[x], df["Roll"], label="Roll", color="orange")
#axes[2].grid(True)
axes[2].set_xlim(pkg_min, pkg_max)
axes[2].set_xticks(np.arange(pkg_min,pkg_max,500))
axes[2].set_xticklabels(axes[2].get_xticks(),rotation=45, ha='right')

axes[2].set_ylim(0,360 )
axes[2].set_yticks(np.arange(-0,360,30))

axes[2].legend()





# axes[2, 3].# plot(df['Tick'], df['Thrust'], label='Thrust', color='red')
#axes[3].plot(df[x], df["Velocity"], label="Velocity", color="orange")
#axes[2].grid(True)
#axes[3].set_xlim(pkg_min, pkg_max)
#axes[2].set_ylim(-1000,1000 )
#axes[3].legend()







# Adjust the spacing between subplots
# plt.subplots_adjust(wspace=0.3, hspace=0.5)

# Save and show the plot
#plt.savefig("AKU1_flight_Data_SD_Card_Euler.png")
plt.show()
