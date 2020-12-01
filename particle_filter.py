import random
import numpy as np
import math
import bisect
import copy
from utils import add_noise

NUM_PARTICLES = 80

class WeightedDistribution(object):

    def __init__(self, particles):
        
        accum = 0.0
        self.particles = particles
        self.distribution = list()
        for particle in self.particles:
            accum += particle.weight
            self.distribution.append(accum)

    def random_select(self):

        try:
            return self.particles[bisect.bisect_left(self.distribution, np.random.uniform(0, 1))]
        except IndexError:
            # When all particles have weights zero
            return None

def euclidean_distance(x1, x2):
    return np.linalg.norm(np.asarray(x1) - np.asarray(x2))

def weight_gaussian_kernel(x1, x2, std = 10):
    distance = euclidean_distance(x1 = x1, x2 = x2)
    return np.exp(-distance ** 2 / (2 * std))

def reweight(particle, func, ref):
    # BEGIN_YOUR_CODE
    raise NotImplementedError
    # END_YOUR_CODE
    
def normalize(particles):
    weight_total = 0
    for p in particles:
        weight_total += p.weight

    if weight_total == 0:
        weight_total = 1e-8
    
    for p in particles:
        p.weight /= weight_total

def resample(particles, minx, maxx, miny, maxy):
    new_particles = []

    distribution = WeightedDistribution(particles=particles)

    for _ in range(len(particles)):
        particle = distribution.random_select()
        if particle is None:
            pos = np.array([np.random.uniform(minx, maxx), np.random.uniform(miny, maxy)])
            orient = np.array([random.random() - 0.5, random.random() - 0.5])
            orient = orient / np.linalg.norm(orient)
            new_particles.append(Particle(pos, orient))
        else:
            p = Particle(copy.deepcopy(particle.pos), copy.deepcopy(particle.orient), noisy=True)
            new_particles.append(p)
    
    return new_particles

def move(particle, theta, speed):
    # BEGIN_YOUR_CODE
    raise NotImplementedError
    # END_YOUR_CODE

def estimate_pose(particles):
    x_accum = 0
    y_accum = 0
    angle_accum = 0
    weight_accum = 0.0
    for p in particles:
        weight_accum += p.weight
        x_accum += p.pos[0] * p.weight
        y_accum += p.pos[1] * p.weight
        angle = int(math.degrees(math.atan2(-p.orient[1], p.orient[0])))
        angle_accum += angle * p.weight
    if weight_accum != 0:
        x_est = x_accum / weight_accum
        y_est = y_accum / weight_accum
        angle_est = int(angle_accum / weight_accum)
        return x_est, y_est, angle_est
    else:
        raise ValueError

class Particle:
    def __init__(self, pos, orient, weight=1.0, noisy=False):
        self.pos = pos
        self.orient = orient
        self.weight = weight
        if noisy:
            std = 3 * 10 * 0.2
            self.pos[0] = add_noise(x=self.pos[0], std=std)
            self.pos[1] = add_noise(x=self.pos[1], std=std)
            while True:
                self.orient[0] = add_noise(x=self.orient[0], std=std*0.01)
                self.orient[1] = add_noise(x=self.orient[1], std=std*0.01)
                if np.linalg.norm(self.orient) >= 1e-8:
                    break
            self.orient = self.orient / np.linalg.norm(self.orient)

class ParticleFilter:
    def __init__(self, num_particles, minx, maxx, miny, maxy):
        self.num_particles = num_particles
        self.minx = minx
        self.maxx = maxx
        self.miny = miny
        self.maxy = maxy
        self.particles = self.initialize_particles(minx, maxx, miny, maxy)
        
    def initialize_particles(self, minx, maxx, miny, maxy):
        # BEGIN_YOUR_CODE
        raise NotImplementedError
        # END_YOUR_CODE
    
    def fix_particle(self, p):
        x = p.pos[0]
        y = p.pos[1]
        p.pos[0] = max(min(x,self.maxx),self.minx)
        p.pos[1] = max(min(y,self.maxy),self.miny)
        return p
    
    def step(self, func, ref, th, new_speed):
        # BEGIN_YOUR_CODE
        raise NotImplementedError
        # END_YOUR_CODE
        
        return x_est, y_est, angle_est
