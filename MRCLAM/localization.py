import gtsam

data = robots_data
data["landmark_groundtruth"] = landmark_groundtruth

def simulation:
    def __init__(data, seed=0):
        """
        including:
            - landmark_groundtruth
                2D numpy array, every row contains (Subject, x [m], y [m], x std-dev [m], y std-dev [m])
            - robot1_groundtruth, ..., robot5_groundtruth
                2D numpy array, every row contains (Time [s], x [m], y [m], orientation [rad])
        data: dictionary containing groundtruth information of MRCLAM dataset, here we use self-generated
        odometry and measurements in order to have better control of the noise
        """
        self.data = data
        self.steps = self.data["robot1_groundtruth"].shape[0]
        self.seed = seed
        self.random_state = np.random.RandomState(self.seed)
        
        self.max_range = 4.0
        self.max_bearing = np.pi / 3.0 # 60 degree
        
        self.sigma_x = 0.01
        self.sigma_y = 0.01
        self.sigma_theta = 0.005
        self.sigma_bearing = 0.005
        self.sigma_range = 0.01
    
    def step(self):
        """
        return:
            dict of (robot numbering -> (odometry, measurements))
                odometry: odometry between two poses (initial pose is returned for the first step)
                observations: dict of (numbering (robot or landmark) -> (bearing, range))
        """
        info = {}
        for i in range(self.steps):
            for j in range(1, n_robots + 1):
                groundtruth = self.data[F"robot{j}_groundtruth"]
                current_pose = gtsam.Pose2(groundtruth[i, 1:4])
                if i == 0:
                    odom = current_pose
                else:
                    prev_pose = gtsam.Pose2(groundtruth[i - 1, 1:4])
                    odom = prev_pose.between(current_pose)
                dx = self.random_state.normal(0.0, self.sigma_x)
                dy = self.random_state.normal(0.0, self.sigma_y)
                dt = self.random_state.normal(0.0, self.sigma_theta)
                odom = odom.compose(gtsam.Pose2(dx, dy, dt))
                
                obs = {}
                for k in range(1, 1 + n_robots + n_landmarks):
                    if k == j:
                        continue
                    elif k < 1 + n_robots:
                        point = gtsam.Point2(self.data[F"robot{k}_groundtruth"][i, 1:3])
                    else:
                        point = gtsam.Point2(self.data["landmark_groundtruth"][k, 1:3])
                    bearing = current_pose.bearing(point).theta()
                    distance = current_pose.range(point)
                    if o < distance < self.max_range and abs(bearing) < self.max_bearing:
                        bearing += self.random_state.normal(0.0, self.sigma_bearing)
                        distance += self.random_state.normal(0.0, self.sigma_range)
                        obs[k] = bearing, distance
                
                info[j] = (odom, obs)
        return info

def X(i, t):
    assert 1 <= i <= n_robots, "robot numbering is out of range"
    name = ['_', 'a', 'b', 'c', 'd', 'e']
    return gtsam.symbol(name[i], t)

def L(j):
    assert n_robots + 1 <= j <= n_landmarks + n_robots + 1, "landmark numbering is out of range"
    return gtsam.symbol('L', j)

def localization(sim):
    isam = gtsam.ISAM2()
    new_factors = gtsam.NonlinearFactorGraph()
    initial_estimate = gtsam.Values()
    
    observed = set()
    for t, info in enumerate(sim.step()):
        for k in info:
            odom, obs = info[k]
            if t == 0:
                    prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
                        [sim.sigma_x, sim.sigma_y, sim.sigma_theta]))
                    prior_factor = gtsam.PriorFactorPose2(k, odom, prior_noise)
                    new_factors.add(prior_factor)
                    initial_estimate.insert(X(k, t), odom)
            else:
                odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
                    [sim.sigma_x, sim.sigma_y, sim.sigma_theta]))
                odom_factor = gtsam.BetweenFactorPoint2(X(k, t - 1), X(k, t), odom, odom_noise)
                new_factors.add(odom_factor)
                estimated_pose = isam.calculateEstimatePose2(X(k, t - 1))
                initial_estimate.insert(X(k, t), estimated_pose.compose(odom))
            
            isam.update(new_factors, initial_estimate)
            new_factors.resize(0)
            initial_estimate.clear()
            
            for numbering, (bearing, distance) in obs.items():
                br_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array(
                    [sim.sigma_bearing, sim.sigma_range]))
                if numbering < 1 + n_robots:
                    br_factor = gtsam.BearingRangeFactorPose2(
                        X(k, t), X(numbering, t), gtsam.Rot2(bearing, distance), br_noise)
                    new_factors.add(br_factor)
                else:
                    br_factor = gtsam.BearingRange2D(
                        X(k, t), L(numbering), gtsam.Rot2(bearing, distance), br_noise)
                    new_factors.add(br_factor)
                    if numbering not in observed:
                        initial_estimate.insert(L(numbering), 
                            gtsam.Point2(sim.data["landmark_groundtruth"][numbering - 1 - n_robots], 1:3))
                        observed.add(numbering)
                        
            isam.update(new_factors, initial_estimate)
            new_factors.resize(0)
            initial_estimate.clear()
    estimated_trajectory = []
    for t in range(sim.steps):
        poses = {}
        for k in range(1, n_robots + 1):
            p = isam.calculateEstimatePose2(X(k, t))
            p = np.array([p.x(), p.y(), p.theta()])
            poses[k] = p
        estimated_trajectory.append(poses)
    return estimated_trajectory
