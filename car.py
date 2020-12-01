import numpy as np
from utils import angle_bw

CAR_CIRCLE_RADIUS = 9
CAR_CIRCLE_DIST = 1.2 * CAR_CIRCLE_RADIUS
STEER_SPEED = 1.0
MAX_SENSOR_RANGE = 50

class CarCircle:
    def __init__(self, x, y, radius):
        self.pos = np.array([x,y])
        self.radius = radius

class Car:
    def __init__(self, x=750.0, y=750.0, vel=np.array([0.0,0.0]), orient=np.array([1.0,0.0]), particle_weight=1.0, noisy=False):
        self.pos = np.array([x,y])
        self.vel = vel
        self.old_vel = self.vel
        self.orient = orient
        self.throttle = False
        self.brake = False
        self.steer_left = False
        self.steer_right = False
        self.iters_since_last_blank = 0
        self.max_speed = 10.0
        self.accel = np.array([0.0,0.0])
        self.accel_coef = 0.15
        self.brake_coef = -0.12
        self.steer_th = -0.07
        self.responses = []
        self.circle_front = CarCircle(x + CAR_CIRCLE_DIST, y, CAR_CIRCLE_DIST)
        self.circle_middle = CarCircle(x,y,CAR_CIRCLE_DIST)
        self.circle_rear = CarCircle(x - CAR_CIRCLE_DIST, y, CAR_CIRCLE_DIST)
        self.particle_weight = particle_weight
    
    def throttle_press(self):
        self.throttle = True
    
    def brake_press(self):
        self.brake = True
    
    def steer_left(self):
        self.steer_left = True
    
    def steer_right(self):
        self.steer_right = True
    
    def steer(self, dir):
        assert dir in ["left","right"]
        
        cur_speed = np.linalg.norm(self.vel)

        if cur_speed < STEER_SPEED:
            magnitude = 0.5 * cur_speed
        else:
            magnitude = 1.0 - (cur_speed / (1.5 * self.max_speed))

        if dir == "left":
            th = magnitude * self.steer_th
        elif dir == "right":
            th = -magnitude * self.steer_th
        
        ox = self.orient[0]
        oy = self.orient[1]
        self.orient[0] = ox * np.cos(th) - oy * np.sin(th)
        self.orient[1] = ox * np.sin(th) + oy * np.cos(th)
        self.orient = self.orient / np.linalg.norm(self.orient)
        
        # slow the car down a little bit
        self.responses.append(-0.0002 * self.vel)
    
    def collision_response(self, contour):
        for i,circle in enumerate([self.circle_front, self.circle_middle, self.circle_rear]):
            for j,cur in enumerate(contour.points):
                prev = contour.points[j-1]

                vec = cur - prev
                posmod = circle.pos - prev
                unit_vec = vec / np.linalg.norm(vec)
                proj = (np.dot(posmod, vec) / np.linalg.norm(vec)) * unit_vec
                dist = np.linalg.norm(posmod - proj)
                minx = min(vec[0],0)
                maxx = max(vec[0],0)
                miny = min(vec[1],0)
                maxy = max(vec[1],0)
                px = proj[0]
                py = proj[1]
                if dist <= circle.radius and minx <= px and px <= maxx and miny <= py and py <= maxy:
                    velproj = (np.dot(self.vel, vec) / np.linalg.norm(vec)) * unit_vec
                    response = (velproj - self.vel) * 1.1
                    self.responses.append(velproj * -0.06) # friction
                    self.responses.append(self.vel * -0.007)
                    self.pos += 1.5 * (circle.radius - dist) * (posmod - proj) / np.linalg.norm(posmod - proj)
                    if i == 0 or i == 2:
                        if i == 0:
                            circlepos = self.circle_front.pos
                            coef = 1
                        elif i == 2:
                            circlepos = self.circle_rear.pos
                            coef = 1
                        oldangle = angle_bw(circlepos - self.circle_middle.pos, self.orient)
                        newangle = angle_bw(circlepos + response - self.circle_middle.pos, self.orient)

                        cur_speed = np.linalg.norm(self.vel)
                        th = coef * np.sign(newangle - oldangle) * np.log(abs(newangle - oldangle)) * (cur_speed / (1.0 * self.max_speed)) * 0.02
                        ox = self.orient[0]
                        oy = self.orient[1]
                        self.orient[0] = ox * np.cos(th) - oy * np.sin(th)
                        self.orient[1] = ox * np.sin(th) + oy * np.cos(th)
                        self.orient = self.orient / np.linalg.norm(self.orient)
    
    def update(self, contour_inner, contour_outer, collision=True):
        self.old_vel = self.vel
        cur_speed = np.linalg.norm(self.vel)
        unit_orient = self.orient / np.linalg.norm(self.orient)
        if cur_speed > 1e-8:
            unit_vel = self.vel / np.linalg.norm(self.vel)
        else:
            unit_vel = unit_orient

        if self.steer_left:
            self.steer("left")
        if self.steer_right:
            self.steer("right")

        # slow the car down a little bit
        self.responses.append(-0.01 * self.vel)
        
        # throttle and brake
        if self.throttle and cur_speed <= self.max_speed:
            new_vec = self.accel_coef * unit_orient
            self.responses.append(new_vec)
        elif self.brake:
            new_vec = self.brake_coef * unit_vel
            #print(new_vec)
            self.responses.append(new_vec)
        
        # push velocity to point in direction of orientation
        if cur_speed >= 0.0001:
            diff = unit_orient - unit_vel
            self.responses.append(0.4 * cur_speed * diff)
        
        if collision:
            self.collision_response(contour_inner)
            self.collision_response(contour_outer)
        
        self.accel = np.sum(self.responses, axis=0)
        self.vel = self.vel + self.accel
        self.pos = self.pos + self.vel
        self.responses = []

        self.circle_front.pos = self.pos + CAR_CIRCLE_DIST * unit_orient
        self.circle_middle.pos = self.pos
        self.circle_rear.pos = self.pos - CAR_CIRCLE_DIST * unit_orient

        self.iters_since_last_blank += 1
        if self.iters_since_last_blank > 1:
            self.throttle = False
            self.brake = False
            self.steer_left = False
            self.steer_right = False
            self.iters_since_last_blank = 0
