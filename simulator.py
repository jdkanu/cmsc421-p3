import numpy as np
from car import Car
from particle_filter import ParticleFilter, NUM_PARTICLES
from kalman_filter import KalmanFilter
from utils import angle_bw
from racetrack import load_racetrack

WORLD_WIDTH = 1400
WORLD_HEIGHT = 800
KF_VAR = 5.0

class Simulator:
    def __init__(self):
        self.racetrack = load_racetrack("data/racetrack.p")
        self.car = Car()
        self.dists = np.array([0,0,0,0])

        self.do_particles = False
        self.pf = None
        self.x_est = None
        self.y_est = None
        self.angle_est = None

        self.do_kf = False
        self.kf = None
        self.meas = None
        self.kf_x = None

        self.r_count = 0
        self.cur_rightness = 0.5

        self.lap_data = []
        self.crossed_start = False
        self.lap_data_old = np.load("data/lap_data.npy")
        self.cur_i = 2
        self.recording = False
        self.replaying = False
    
    def init_particles(self):
        self.do_particles = True
        self.pf = ParticleFilter(NUM_PARTICLES,0,WORLD_WIDTH,0,WORLD_HEIGHT)
    
    def stop_particles(self):
        self.do_particles = False
        self.pf = None
    
    def init_kalman(self):
        self.do_kf = True
        self.kf = KalmanFilter(KF_VAR)
    
    def stop_kalman(self):
        self.do_kf = False
        self.kf = None
    
    def toggle_particles(self):
        if self.do_particles:
            self.stop_particles()
        else:
            self.init_particles()
    
    def toggle_kalman(self):
        if self.do_kf:
            self.stop_kalman()
        else:
            self.init_kalman()
    
    def loop(self):
        if self.recording:
            self.lap_data.append(np.append(self.car.pos, self.car.orient))
            datum = self.racetrack.progress(self.car)
            progress = datum[0]
            if 1.1 < progress and progress < 1.5 and not self.crossed_start:
                self.crossed_start = True
            if 0.5 < progress and progress < 1.0 and self.crossed_start:
                np.save("data/lap_data.npy", np.array(self.lap_data))
                print("finished")
                self.crossed_start = False
        if self.replaying:
            dp = self.lap_data_old[self.cur_i]
            pos = dp[0:2]
            orient = dp[2:]

            dp_1 = self.lap_data_old[self.cur_i - 1]
            pos_1 = dp_1[0:2]

            dp_1 = self.lap_data_old[self.cur_i - 2]
            pos_2 = dp_1[0:2]

            self.car.pos = pos
            self.car.vel = pos - pos_1
            self.car.old_vel = pos_1 - pos_2
            self.car.orient = orient

            self.cur_i += 1
        
        if not self.replaying:
            self.car.update(self.racetrack.contour_inner, self.racetrack.contour_outer)
        d_orient = angle_bw(self.car.vel, self.car.old_vel) * np.pi / 180.0

        #######################################################################################################################################
        # common variable
        self.dists = self.racetrack.read_distances(self.car.pos[0], self.car.pos[1], noisy=False, std=2)
        
        #######################################################################################################################################
        # opponent
        do_steer = False
        if do_steer:
            datum = self.racetrack.progress(self.car)
            self.r_count += 1
            if self.r_count % 100 == 0:
                if self.cur_rightness == 0.5 or self.r_count == 100:
                    self.cur_rightness = 0.5
                else:
                    self.cur_rightness = 0.5
                    #self.cur_rightness = np.random.uniform(0.3,0.7)
                print(self.cur_rightness)
            rightness = datum[1]
            angle = datum[2]
            speed = datum[3]
            ang_steer = 30 * np.exp(-0.2*speed)# * (2 - np.exp(-2*np.abs(self.cur_rightness - rightness)))
            if rightness > self.cur_rightness:
                if angle > ang_steer:
                    self.car.steer("right")
                else:
                    self.car.steer("left")
            else:
                if angle < -ang_steer:
                    self.car.steer("left")
                else:
                    self.car.steer("right")
        
        #######################################################################################################################################
        # kalman filter
        if self.do_kf:
            self.meas = self.car.pos + np.random.normal(0,KF_VAR,2)
            self.kf_x = self.kf.get_x(self.meas)

        #######################################################################################################################################
        # particle filtering
        if self.do_particles:
            self.x_est, self.y_est, self.angle_est = self.pf.step(self.racetrack.read_distances, self.dists, d_orient, np.linalg.norm(self.car.vel))
