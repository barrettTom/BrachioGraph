#!/usr/bin/python3

from adafruit_servokit import ServoKit

from time import sleep
import readchar
import math
import json
import tqdm

class Bonnet:
    def __init__(self):
        self.kit = ServoKit(channels=16)
        self.last_angles = [0, 0, 0]

    def set_angle(self, i, a):
        if i == 0:
            a += 120
        self.kit.servo[i].angle = a
        self.last_angles[i] = a

    def set_angles(self, a0, a1):
        self.set_angle(0, a0)
        self.set_angle(1, a1)

    def get_angles(self):
        return (self.last_angles[0], self.last_angles[1])

class BrachioGraph:
    def __init__(
        self,
        inner_arm,                  # the lengths of the arms
        outer_arm,
        wait=None,
        bounds=None,                # the maximum rectangular drawing area
        angle_up=0,                 # pulse-widths for pen up/down
        angle_down=10,
    ):

        # set the pantograph geometry
        self.INNER_ARM = inner_arm
        self.OUTER_ARM = outer_arm

        # the box bounds describe a rectangle that we can safely draw in
        self.bounds = bounds

        self.rpi = Bonnet()

        # Initialise the pantograph with the motors in the centre of their travel
        self.rpi.set_angle(0, -90)
        sleep(0.3)
        self.rpi.set_angle(1, 90)
        sleep(0.3)

        # by default we use a wait factor of 0.1 for accuracy
        self.wait = wait or .1

        # create the pen object, and make sure the pen is up
        self.pen = Pen(bg=self, rpi=self.rpi, angle_up=angle_up, angle_down=angle_down)

        # Set the x and y position state, so it knows its current x/y position.
        self.current_x = -self.INNER_ARM
        self.current_y = self.OUTER_ARM

    # ----------------- drawing methods -----------------
    def plot_file(self, filename="", wait=0, interpolate=10, bounds=None):

        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "File plotting is only possible when BrachioGraph.bounds is set."

        with open(filename, "r") as line_file:
            lines = json.load(line_file)

        self.plot_lines(lines=lines, wait=wait, interpolate=interpolate, bounds=bounds, flip=True)


    def plot_lines(self, lines=[], wait=0, interpolate=10, rotate=False, flip=False, bounds=None):

        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "Line plotting is only possible when BrachioGraph.bounds is set."

        lines = self.rotate_and_scale_lines(lines=lines, bounds=bounds, flip=True)

        for line in tqdm.tqdm(lines, desc="Lines", leave=False):
            x, y = line[0]

            # only if we are not within 1mm of the start of the line, lift pen and go there
            if (round(self.current_x, 1), round(self.current_y, 1)) != (round(x, 1), round(y, 1)):
                self.xy(x, y, wait=wait, interpolate=interpolate)

            for point in tqdm.tqdm(line[1:], desc="Segments", leave=False):
                x, y = point
                self.draw(x, y, wait=wait, interpolate=interpolate)

        self.park()


    def draw_line(self, start=(0, 0), end=(0, 0), wait=0, interpolate=10, both=False):
        wait = wait or self.wait

        start_x, start_y = start
        end_x, end_y = end

        self.pen.up()
        self.xy(x=start_x, y=start_y, wait=wait, interpolate=interpolate)

        self.pen.down()
        self.draw(x=end_x, y=end_y, wait=wait, interpolate=interpolate)

        if both:
            self.draw(x=start_x, y=start_y, wait=wait, interpolate=interpolate)

        self.pen.up()


    def draw(self, x=0, y=0, wait=0, interpolate=10):
        wait = wait or self.wait

        self.xy(x=x, y=y, wait=wait, interpolate=interpolate, draw=True)

    # ----------------- line-processing methods -----------------

    def rotate_and_scale_lines(self, lines=[], rotate=False, flip=False, bounds=None):
        rotate, x_mid_point, y_mid_point, box_x_mid_point, box_y_mid_point, divider = self.analyse_lines(
            lines=lines, rotate=rotate, bounds=bounds
        )

        for line in lines:
            for point in line:
                if rotate:
                    point[0], point[1] = point[1], point[0]

                x = point[0]
                x = x - x_mid_point         # shift x values so that they have zero as their mid-point
                x = x / divider             # scale x values to fit in our box width
                x = x + box_x_mid_point     # shift x values so that they have the box x midpoint as their endpoint

                if flip ^ rotate:
                    x = -x

                y = point[1]
                y = y - y_mid_point
                y = y / divider
                y = y + box_y_mid_point

                point[0], point[1] = x, y

        return lines


    def analyse_lines(self, lines=[], rotate=False, bounds=None):

        # lines is a tuple itself containing a number of tuples, each of which contains a number of 2-tuples
        #
        # [                                                                                     # |
        #     [                                                                                 # |
        #         [3, 4],                               # |                                     # |
        #         [2, 4],                               # |                                     # |
        #         [1, 5],  #  a single point in a line  # |  a list of points defining a line   # |
        #         [3, 5],                               # |                                     # |
        #         [3, 7],                               # |                                     # |
        #     ],                                                                                # |
        #     [                                                                                 # |  all the lines
        #         [...],                                                                        # |
        #         [...],                                                                        # |
        #     ],                                                                                # |
        #     [                                                                                 # |
        #         [...],                                                                        # |
        #         [...],                                                                        # |
        #     ],                                                                                # |
        # ]                                                                                     # |

        # First, we create a pair of empty sets for all the x and y values in all of the lines of the plot data.

        x_values_in_lines = set()
        y_values_in_lines = set()

        # Loop over each line and all the points in each line, to get sets of all the x and y values:

        for line in lines:

            x_values_in_line, y_values_in_line = zip(*line)

            x_values_in_lines.update(x_values_in_line)
            y_values_in_lines.update(y_values_in_line)

        # Identify the minimum and maximum values.

        min_x, max_x = min(x_values_in_lines), max(x_values_in_lines)
        min_y, max_y = min(y_values_in_lines), max(y_values_in_lines)

        # Identify the range they span.

        x_range, y_range = max_x - min_x, max_y - min_y
        box_x_range, box_y_range = bounds[2] - bounds[0], bounds[3] - bounds[1]

        # And their mid-points.

        x_mid_point, y_mid_point = (max_x + min_x) / 2, (max_y + min_y) / 2
        box_x_mid_point, box_y_mid_point = (bounds[0] + bounds[2]) / 2, (bounds[1] + bounds[3]) / 2

        # Get a 'divider' value for each range - the value by which we must divide all x and y so that they will
        # fit safely inside the drawing range of the plotter.

        # If both image and box are in portrait orientation, or both in landscape, we don't need to rotate the plot.

        if (x_range >= y_range and box_x_range >= box_y_range) or (x_range <= y_range and box_x_range <= box_y_range):

            divider = max((x_range / box_x_range), (y_range / box_y_range))
            rotate = False

        else:

            divider = max((x_range / box_y_range), (y_range / box_x_range))
            rotate = True
            x_mid_point, y_mid_point = y_mid_point, x_mid_point

        return rotate, x_mid_point, y_mid_point, box_x_mid_point, box_y_mid_point, divider


    # ----------------- test pattern methods -----------------

    def test_pattern(self, bounds=None, wait=0, interpolate=10, repeat=1):

        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "Plotting a test pattern is only possible when BrachioGraph.bounds is set."

        for r in tqdm.tqdm(tqdm.trange(repeat, desc='Iteration'), leave=False):

            for y in range(bounds[1], bounds[3], 2):

                self.xy(bounds[0],   y,     wait, interpolate)
                self.draw(bounds[2], y,     wait, interpolate)
                self.xy(bounds[2],   y + 1, wait, interpolate)
                self.draw(bounds[0], y + 1, wait, interpolate)

        self.park()


    def vertical_lines(self, bounds=None, lines=4, wait=0, interpolate=10, repeat=1, reverse=False, both=False):

        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "Plotting a test pattern is only possible when BrachioGraph.bounds is set."

        if not reverse:
            top_y =    self.bounds[1]
            bottom_y = self.bounds[3]
        else:
            bottom_y = self.bounds[1]
            top_y =    self.bounds[3]

        step = (self.bounds[2] - self.bounds[0]) /  lines
        x = self.bounds[0]
        while x <= self.bounds[2]:
            self.draw_line((x, top_y), (x, bottom_y), interpolate=interpolate, both=both)
            x = x + step

        self.park()


    def horizontal_lines(self, bounds=None, lines=4, wait=0, interpolate=10, repeat=1, reverse=False, both=False):

        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "Plotting a test pattern is only possible when BrachioGraph.bounds is set."

        if not reverse:
            min_x = self.bounds[0]
            max_x = self.bounds[2]
        else:
            max_x = self.bounds[0]
            min_x = self.bounds[2]

        step = (self.bounds[3] - self.bounds[1]) / lines
        y = self.bounds[1]
        while y <= self.bounds[3]:
            self.draw_line((min_x, y), (max_x, y), interpolate=interpolate, both=both)
            y = y + step

        self.park()


    def grid_lines(self, bounds=None, lines=4, wait=0, interpolate=10, repeat=1, reverse=False, both=False):

        self.vertical_lines(
            bounds=bounds, lines=lines, wait=wait, interpolate=interpolate, repeat=repeat, reverse=reverse, both=both
            )
        self.horizontal_lines(
            bounds=bounds, lines=lines, wait=wait, interpolate=interpolate, repeat=repeat, reverse=reverse, both=both
            )


    def box(self, bounds=None, wait=0, interpolate=10, repeat=1, reverse=False):

        wait = wait or self.wait
        bounds = bounds or self.bounds

        if not bounds:
            return "Box drawing is only possible when BrachioGraph.bounds is set."

        self.xy(bounds[0], bounds[1], wait, interpolate)

        for r in tqdm.tqdm(tqdm.trange(repeat), desc='Iteration', leave=False):

            if not reverse:

                self.draw(bounds[2], bounds[1], wait, interpolate)
                self.draw(bounds[2], bounds[3], wait, interpolate)
                self.draw(bounds[0], bounds[3], wait, interpolate)
                self.draw(bounds[0], bounds[1], wait, interpolate)

            else:

                self.draw(bounds[0], bounds[3], wait, interpolate)
                self.draw(bounds[2], bounds[3], wait, interpolate)
                self.draw(bounds[2], bounds[1], wait, interpolate)
                self.draw(bounds[0], bounds[1], wait, interpolate)

        self.park()


    # ----------------- pen-moving methods -----------------

    def xy(self, x=0, y=0, wait=0, interpolate=10, draw=False):
        # Moves the pen to the xy position; optionally draws

        wait = wait or self.wait

        if draw:
            self.pen.down()
        else:
            self.pen.up()

        (angle_1, angle_2) = self.xy_to_angles(x, y)

        if (angle_1, angle_2) == self.rpi.get_angles():
            # ensure the pantograph knows its x/y positions
            self.current_x = x
            self.current_y = y

            return

        # we assume the pantograph knows its x/y positions - if not, there could be
        # a sudden movement later

        # calculate how many steps we need for this move, and the x/y length of each
        (x_length, y_length) = (x - self.current_x, y - self.current_y)

        length = math.sqrt(x_length ** 2 + y_length **2)

        no_of_steps = int(length * interpolate) or 1

        if no_of_steps < 100:
            disable_tqdm = True
        else:
            disable_tqdm = False

        (length_of_step_x, length_of_step_y) = (x_length/no_of_steps, y_length/no_of_steps)

        for step in tqdm.tqdm(range(no_of_steps), desc='Interpolation', leave=False, disable=disable_tqdm):

            self.current_x = self.current_x + length_of_step_x
            self.current_y = self.current_y + length_of_step_y

            angle_1, angle_2 = self.xy_to_angles(self.current_x, self.current_y)

            self.set_angles(angle_1, angle_2)

            if step + 1 < no_of_steps:
                sleep(length * wait/no_of_steps)

        sleep(length * wait/10)


    def set_angles(self, angle_1=0, angle_2=0):
        # moves the servo motor
        self.rpi.set_angles(angle_1, angle_2)

    #  ----------------- hardware-related methods -----------------
    def park(self):
        self.pen.up()
        self.xy(-self.INNER_ARM, self.OUTER_ARM)
        sleep(1)

    # ----------------- trigonometric methods -----------------

    # Every x/y position of the plotter corresponds to a pair of angles of the arms. These methods
    # calculate:
    #
    # the angles required to reach any x/y position
    # the x/y position represented by any pair of angles

    def xy_to_angles(self, x=0, y=0):

        # convert x/y co-ordinates into motor angles

        hypotenuse = math.sqrt(x**2+y**2)

        if hypotenuse > self.INNER_ARM + self.OUTER_ARM:
            raise Exception(f"Cannot reach {hypotenuse}; total arm length is {self.INNER_ARM + self.OUTER_ARM}")

        hypotenuse_angle = math.asin(x/hypotenuse)

        inner_angle = math.acos(
            (hypotenuse**2+self.INNER_ARM**2-self.OUTER_ARM**2)/(2*hypotenuse*self.INNER_ARM)
        )
        outer_angle = math.acos(
            (self.INNER_ARM**2+self.OUTER_ARM**2-hypotenuse**2)/(2*self.INNER_ARM*self.OUTER_ARM)
        )

        shoulder_motor_angle = hypotenuse_angle - inner_angle
        elbow_motor_angle = math.pi - outer_angle

        return (math.degrees(shoulder_motor_angle), math.degrees(elbow_motor_angle))


    def angles_to_xy(self, shoulder_motor_angle, elbow_motor_angle):

        # convert motor angles into x/y co-ordinates

        elbow_motor_angle = math.radians(elbow_motor_angle)
        shoulder_motor_angle = math.radians(shoulder_motor_angle)

        hypotenuse = math.sqrt(
            (self.INNER_ARM ** 2 + self.OUTER_ARM ** 2 - 2 * self.INNER_ARM * self.OUTER_ARM * math.cos(
                math.pi - elbow_motor_angle)
            )
        )
        base_angle = math.acos(
            (hypotenuse ** 2 + self.INNER_ARM ** 2 - self.OUTER_ARM ** 2) / (2 * hypotenuse * self.INNER_ARM)
        )
        inner_angle = base_angle + shoulder_motor_angle

        x = math.sin(inner_angle) * hypotenuse
        y = math.cos(inner_angle) * hypotenuse

        return(x, y)

    # ----------------- manual driving methods -----------------

    def drive(self):
        a0, a1 = self.get_angles()

        self.set_angles(a0, a1)

        while True:
            key = readchar.readchar()

            if key == "0":
                return
            elif key=="a":
                a0 = a0 - 10
            elif key=="s":
                a0 = a0 + 10
            elif key=="A":
                a0 = a0 - 1
            elif key=="S":
                a0 = a0 + 1
            elif key=="k":
                a1 = a1 - 10
            elif key=="l":
                a1 = a1 + 10
            elif key=="K":
                a1 = a1 - 1
            elif key=="L":
                a1 = a1 + 1

            print(a0, a1)

            self.set_angles(a0, a1)


    def drive_xy(self):

        # move the pen up/down and left/right using the keyboard

        while True:
            key = readchar.readchar()

            if key == "0":
                return
            elif key=="a":
                self.current_x = self.current_x - 1
            elif key=="s":
                self.current_x = self.current_x + 1
            elif key=="A":
                self.current_x = self.current_x - .1
            elif key=="S":
                self.current_x = self.current_x + .1
            elif key=="k":
                self.current_y = self.current_y - 1
            elif key=="l":
                self.current_y = self.current_y + 1
            elif key=="K":
                self.current_y = self.current_y - .1
            elif key=="L":
                self.current_y = self.current_y + .1

            print(self.current_x, self.current_y)

            self.xy(self.current_x, self.current_y)

    @property
    def bl(self):
        return (self.bounds[0], self.bounds[1])

    @property
    def tl(self):
        return (self.bounds[0], self.bounds[3])

    @property
    def tr(self):
        return (self.bounds[2], self.bounds[3])

    @property
    def br(self):
        return (self.bounds[2], self.bounds[1])

class Pen:
    def __init__(self, bg, rpi, angle_up=0, angle_down=10, pin=2, transition_time=0.25):

        self.bg = bg
        self.pin = pin
        self.angle_up = angle_up
        self.angle_down = angle_down
        self.transition_time = transition_time

        self.rpi = rpi

        self.up()
        sleep(0.3)
        self.down()
        sleep(0.3)
        self.up()
        sleep(0.3)

    def down(self):
        self.rpi.set_angle(self.pin, self.angle_down)
        sleep(self.transition_time)

    def up(self):
        self.rpi.set_angle(self.pin, self.angle_up)
        sleep(self.transition_time)
