import SumoNetVis
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.ticker as ticker
# some visualization params
tick_spacing = 20

# Plot Sumo Network
net = SumoNetVis.Net("cross.net.xml")
net.plot()

# Plot trajectories
trajectories = SumoNetVis.Trajectories("fcd-output.xml")
for trajectory in trajectories:
        # trajectory.assign_colors_speed()
        trajectory.point_plot_kwargs["ms"] = 9  # set marker size. Can set any kwargs taken by matplotlib.pyplot.plot().
fig, ax = plt.gcf(), plt.gca()
a = animation.FuncAnimation(fig, trajectories.plot_points, frames=trajectories.timestep_range() + 2, repeat=False,
                            interval=200*trajectories.timestep, fargs=(ax,), blit=True)

# Show figure
plt.axis('square')
plt.gca().xaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
plt.gca().yaxis.set_major_locator(ticker.MultipleLocator(tick_spacing))
plt.show()
