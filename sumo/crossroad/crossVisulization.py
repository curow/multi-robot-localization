import SumoNetVis
import matplotlib.pyplot as plt
# Plot Sumo Network
net = SumoNetVis.Net("cross.net.xml")
net.plot()
# Plot trajectories
trajectories = SumoNetVis.Trajectories("fcd-output.xml")
trajectories["0"].assign_colors_speed()
trajectories["0"].plot()
# Show figure
plt.show()
