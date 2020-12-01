import math
from tkinter import *
from PIL import Image, ImageTk
from utils import load_image
from simulator import Simulator, WORLD_WIDTH, WORLD_HEIGHT
from racetrack import RaceTrack, Contour, Horizontals, load_racetrack

MAX_COUNT_SINCE = 5

KEYS = {"up":111, "down":116, "left":113, "right":114}

class App(Tk):
    TICK_RATE = 40

    def __init__(self):
        Tk.__init__(self, None, baseName=None,
                    className='Tk', useTk=1, sync=0, use=None)
        self.draw_extra = False
        self.__canvas = Canvas(self, width=WORLD_WIDTH, height=WORLD_HEIGHT)
        self.__canvas.pack()
        self.__canvas.configure(background="red")
        self.title("Simulator")
        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)
        self.history_chars = []
        self.history_keycodes = []
        _, self.bg = load_image("data/track.png")
        
        img_car_blue, _ = load_image("data/car_blue.png")
        self.car_blue_imgs = []
        for i in range(-180,180):
            imgtk = ImageTk.PhotoImage(img_car_blue.rotate(i))
            self.car_blue_imgs.append(imgtk)

        img_green_arrow, _ = load_image("data/green_arrow.png")
        self.green_arrow_imgs = []
        for i in range(-180,180):
            imgtk = ImageTk.PhotoImage(img_green_arrow.rotate(i))
            self.green_arrow_imgs.append(imgtk)

        self.count_since = 0
        self.max_count_since = MAX_COUNT_SINCE

        self.simulator = Simulator()
    
    def keyup(self, e):
        if e.keycode in [111,116,113,114]:
            if e.keycode in self.history_keycodes:
                self.history_keycodes.pop(self.history_keycodes.index(e.keycode))
        elif e.char in self.history_chars:
            self.history_chars.pop(self.history_chars.index(e.char))

    def keydown(self, e):
        #print(e.char)
        if e.keycode in [111,116,113,114]:
            if not e.keycode in self.history_keycodes:
                self.history_keycodes.append(e.keycode)
        elif not e.char in self.history_chars:
            self.history_chars.append(e.char)

    def process_input(self):
        self.count_since += 1
        self.count_since = min(self.count_since, self.max_count_since)
        if not self.simulator.replaying:
            if KEYS["up"] in self.history_keycodes:
                self.simulator.car.throttle_press()
            if KEYS["down"] in self.history_keycodes:
                self.simulator.car.brake_press()
            if KEYS["left"] in self.history_keycodes:
                self.simulator.car.steer("left")
            if KEYS["right"] in self.history_keycodes:
                self.simulator.car.steer("right")
        if "p" in self.history_chars:
            if self.count_since >= self.max_count_since:
                self.simulator.toggle_particles()
                self.count_since = 0
        if "k" in self.history_chars:
            if self.count_since >= self.max_count_since:
                self.simulator.toggle_kalman()
                self.count_since = 0
        if "o" in self.history_chars:
            if self.count_since >= self.max_count_since:
                self.draw_extra = not self.draw_extra
                self.count_since = 0
    
    def __loop(self):
        #######################################################################################################################################
        # loop overhead
        self.after(App.TICK_RATE, self.__loop)
        #######################################################################################################################################
        # update simulator
        self.process_input()
        self.simulator.loop()
        
        #######################################################################################################################################
        # graphics
        self.__canvas.delete(ALL)
        self.__canvas.create_image(0,0,image=self.bg,anchor=NW,tag="bg")

        car = self.simulator.car
        racetrack = self.simulator.racetrack
        
        car_angle = int(math.degrees(math.atan2(-car.orient[1], car.orient[0])))
        self.__canvas.create_image(car.pos[0], car.pos[1], image=self.car_blue_imgs[car_angle + 180])

        # draw grid and occupancy
        if self.draw_extra:
            for i in range(80):
                self.__canvas.create_line(
                    0, 10 * i, 1400, 10 * i, fill="gray")
            for i in range(140):
                self.__canvas.create_line(
                    10 * i, 0, 10 * i, 800, fill="gray")
            for i in range(140):
                for j in range(80):
                    if racetrack.occupancy[i,j] != 0:
                        self.__canvas.create_rectangle(i * 10, j * 10, i * 10 + 10, j * 10 + 10, fill="black")

        sensor_color = "red"
        sensor_width = 2
        if self.draw_extra:
            dists = self.simulator.dists
            self.__canvas.create_line(car.pos[0], car.pos[1], car.pos[0], car.pos[1]-dists[0], fill=sensor_color, width=sensor_width)
            self.__canvas.create_line(car.pos[0], car.pos[1], car.pos[0], car.pos[1]+dists[1], fill=sensor_color, width=sensor_width)
            self.__canvas.create_line(car.pos[0], car.pos[1], car.pos[0]-dists[2], car.pos[1], fill=sensor_color, width=sensor_width)
            self.__canvas.create_line(car.pos[0], car.pos[1], car.pos[0]+dists[3], car.pos[1], fill=sensor_color, width=sensor_width)
        
        draw_particles = self.draw_extra
        if draw_particles:
            if self.simulator.do_particles:
                for p in self.simulator.pf.particles:
                    self.__canvas.create_oval(p.pos[0]-2,p.pos[1]-2,p.pos[0]+2,p.pos[1]+2,fill="red")
        
        if self.simulator.do_particles:
            self.__canvas.create_image(self.simulator.x_est, self.simulator.y_est, image=self.green_arrow_imgs[self.simulator.angle_est + 180])
        
        if self.simulator.do_kf:
            measx = self.simulator.meas[0]
            measy = self.simulator.meas[1]
            self.__canvas.create_oval(measx-4,measy-4,measx+4,measy+4,fill="red")
            kfx = self.simulator.kf_x[0]
            kfy = self.simulator.kf_x[1]
            self.__canvas.create_oval(kfx-4,kfy-4,kfx+4,kfy+4,fill="lime green")
        
        
    def mainloop(self, n=0):
        self.__loop()
        Tk.mainloop(self, n)

def main():
    if True:
        App().mainloop()
    else:
        sim = Simulator()
        #sim.toggle_kalman()
        sim.toggle_particles()
        while True:
            sim.loop()
            print(sim.x_est)


if __name__ == "__main__":
    main()
