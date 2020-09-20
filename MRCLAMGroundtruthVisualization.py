import numpy as np
import pandas as pd
import plotly.express as px
import plotly.graph_objects as go

# choose dataset from 1 to 9
data_num = 1
# number robots is fixed in every dataset
n_robots = 5
# set sample time to process data
sample_time = 0.02
# set visualization timesteps skip
timesteps_per_frame = 100
# draw robots measurements flag
draw_measurements = True
# if measurements ratio is set to high, animation can take a very long time to show
measurements_ratio = 0.05

def textread(filepath):
    """
    read from MRCLAM data in which every line
    contains equal number of values seperated by
    some whitespaces.
    """
    return np.array(pd.read_csv(filepath, 
                       sep = "\s+|\t+|\s+\t+|\t+\s+",
                       header=None,
                       comment='#',
                       engine='python'))
    
root = F'../MRCLAM_Dataset{data_num}/'
print('Parsing Dataset...')

print('\nReading barcode numbers...')
barcodes = textread(root + 'Barcodes.dat')
barcodes = np.array(barcodes)
print('Barcodes: (Subject, Barcode)')

print('\nReading landmark groundtruth...')
landmark_groundtruth = textread(root + 'Landmark_Groundtruth.dat')
n_landmarks = landmark_groundtruth.shape[0]
print(F'Landmark Groundtruth({landmark_groundtruth.shape}): (Subject, x [m], y [m], x std-dev [m], y std-dev [m])')


robots_data = {}
print('\nReading all robots data...')
print('robot groundtruth data((N, 4)): (Time [s], x [m], y [m], orientation [rad])')
print('robot odometry data((N, 3)): (Time [s], forward velocity [m/s], angular velocity [rad/s])')
print('robot measurement data((N, 3)): (Time [s], Subject, range [m], bearing [rad])\n')
for i in range(1, n_robots + 1):
    print(F'Reading robot{i} groundtruth...')
    robots_data[F'robot{i}_groundtruth'] = textread(root + F'Robot{i}_Groundtruth.dat')
    print(F'Reading robot{i} odometry...')
    robots_data[F'robot{i}_odometry'] = textread(root + F'Robot{i}_Odometry.dat')
    print(F'Reading robot{i} measurements...')
    robots_data[F'robot{i}_measurements'] = textread(root + F'Robot{i}_Measurement.dat')
    print('\n')

print('Parsing Complete')

# get the start and end time for the whole dataset
min_time, max_time = float('inf'), float('-inf')
for i in range(1, n_robots + 1):
    min_time = min(min_time, robots_data[F'robot{i}_groundtruth'][0, 0])
    max_time = max(max_time, robots_data[F'robot{i}_groundtruth'][-1, 0])
for data in robots_data.values():
    data[:, 0] -= min_time
total_time = max_time - min_time
timesteps = int(total_time // sample_time + 1)
print(F'start time: {min_time} [s]\nend time: {max_time} [s]\ntotal time: {total_time} [s]')
print(F'sampling time is {sample_time} [s] ({1 / sample_time} [Hz])')
print(F'Discretize to {timesteps} timesteps')

for data_name in robots_data:
    print(F'Processing {data_name}')
    data = robots_data[data_name]
    if 'groundtruth' in data_name or 'odometry' in data_name:
        # linear interpolation for groundtruth and odometry data
        rows, cols = data.shape
        sampling_data = np.zeros((timesteps, cols))
        k, i = 0, 0
        for k in range(timesteps):
            t = k * sample_time
            sampling_data[k, 0] = t
            while i < rows - 1 and data[i, 0] <= t:
                i += 1
            if i == 0 or i == rows - 1:
                if 'groundtruth' in data_name:
                    sampling_data[k, 1:] = data[i, 1:]
            else:
                p = (t - data[i - 1, 0]) / (data[i, 0] - data[i - 1, 0])
                for c in range(1, cols):
                    sampling_data[k, c] = p * (data[i, c] - data[i - 1, c]) + data[i - 1, c]
        robots_data[data_name] = sampling_data
    else:
        # for measurements, just round the measurement time
        # to nearest sampling time, and keep other data untouched
        data[:, 0] = np.round(data[:, 0] / sample_time) * sample_time

# make an empty figure
fig_dict = {
    "data": [],
    "layout": {},
    "frames": []
}

# fill in most of layout
fig_dict["layout"]["xaxis"] = {"range": [-2, 6], "title": "x [m]"}
fig_dict["layout"]["yaxis"] = {"range": [-6, 7], "title": "y [m]"}
fig_dict["layout"]["hovermode"] = "closest"
fig_dict["layout"]["updatemenus"] = [
    {
        "buttons": [
            {
                "args": [
                         None,
                         {
                             "frame": {"duration": 50, "redraw": False},
                             "fromcurrent": True,
                             "transition": {"duration": 0, "easing": "quadratic-in-out"}
                         }
                        ],
                "label": "Play",
                "method": "animate"
            },
            {
                "args": [
                         [None],
                         {
                             "frame": {"duration": 0, "redraw": False},
                             "mode": "immediate",
                             "transition": {"duration": 0}
                         }
                        ],
                "label": "Pause",
                "method": "animate"
            }
        ],
        "direction": "left",
        "pad": {"r": 10, "t": 87},
        "showactive": False,
        "type": "buttons",
        "x": 0.1,
        "xanchor": "right",
        "y": 0,
        "yanchor": "top"
    }
]

sliders_dict = {
    "active": 0,
    "yanchor": "top",
    "xanchor": "left",
    "currentvalue": {
        "font": {"size": 20},
        "prefix": F"Time(1 unit for {timesteps_per_frame * sample_time} second):",
        "visible": True,
        "xanchor": "right"
    },
    "transition": {"duration": 300, "easing": "cubic-in-out"},
    "pad": {"b": 10, "t": 50},
    "len": 0.9,
    "x": 0.1,
    "y": 0,
    "steps": []
}

colors = px.colors.qualitative.Plotly

def robot_pose(i, k):
    poses = robots_data[F'robot{i}_groundtruth']
    x, y, theta = poses[k, 1:]
    location = {
        "x": [x],
        "y": [y],
        "mode": "markers",
        "marker": {"size": 13, "color": colors[i], "line": {"color": "black", "width": 2}},
        "name": F"robot{i}"
    }
    orientation_length = 0.3
    orientation = {
        "x": [x, x + orientation_length * np.cos(theta)],
        "y": [y, y + orientation_length * np.sin(theta)],
        "mode": "lines",
        "line": {"color": "black", "width": 2.5},
        "showlegend": False
    }
    return [location, orientation] 

def robot_measurement(i, k):
    measurements = robots_data[F'robot{i}_measurements']
    poses = robots_data[F'robot{i}_groundtruth']
    x, y, theta = poses[k, 1:4]
    xs, ys = [], []
    for offset in range(int(timesteps_per_frame * measurements_ratio)):
        measurment_x, measurment_y = [], []
        time = (k - offset) * sample_time
        measurements_time = measurements[:, 0]
        index = np.searchsorted(measurements_time, time)
        while index < len(measurements_time) and measurements[index, 0] == time:
            distance, bearing = measurements[index, 2:4]
            measurment_x.extend([x, x + distance * np.cos(bearing + theta), None])
            measurment_y.extend([y, y + distance * np.sin(bearing + theta), None])
            index += 1
        remain = max_num_measurements * 3 - len(measurment_x)
        if remain > 0:
            measurment_x.extend([None for _ in range(remain)])
            measurment_y.extend([None for _ in range(remain)])
        xs.extend(measurment_x)
        ys.extend(measurment_y)
    return {
        "x": xs,
        "y": ys,
        "mode": "lines",
        "line": {"color": colors[i], "width": 1.5},
        "showlegend": False
    }

# robots initial location
for i in range(1, n_robots + 1):
    fig_dict["data"].extend(robot_pose(i, 0))
    if draw_measurements:
        fig_dict["data"].append(robot_measurement(i, 0))

landmarks = dict(
    mode='markers',
    x=landmark_groundtruth[:, 1],
    y=landmark_groundtruth[:, 2],
    marker=dict(
        color='LightSkyBlue',
        size=8,
        line=dict(
            color='MediumPurple',
            width=2
        )
    ),
    name='Landmarks'
)

fig_dict["data"].append(landmarks)

num_frames = timesteps // timesteps_per_frame
for k in trange(num_frames):
    time = k * timesteps_per_frame * sample_time
    frame = {"data": [], "name": str(k)}
    for i in range(1, n_robots + 1):
        frame["data"].extend(robot_pose(i, k * timesteps_per_frame))
        if draw_measurements:
            frame["data"].append(robot_measurement(i, k * timesteps_per_frame))
    frame["data"].append(landmarks)
        
    fig_dict["frames"].append(frame)
    slider_step = {
        "args": [
            [k],
            {
                "frame": {"duration": 20, "redraw": False},
                "mode": "immediate",
                "transition": {"duration": 0}
            }
        ],
        "label": k,
        "method": "animate"
    }
    sliders_dict["steps"].append(slider_step)
fig_dict["layout"]["sliders"] = [sliders_dict]

fig = go.Figure(fig_dict)
fig.update_layout(
    width = 700,
    height = 700,
    title = "Robots Groudtruth Data Visulization"
)
fig.update_yaxes(
    scaleanchor = "x",
    scaleratio = 1,
)
fig.show()
