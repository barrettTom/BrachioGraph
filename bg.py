#!/usr/bin/python3

from time import sleep
from brachiograph import BrachioGraph
from linedraw import vectorise, image_to_json

bg = BrachioGraph(
    # the lengths of the arms
    inner_arm=8,
    outer_arm=8,
    # the drawing area
    bounds=(-8, 4, 8, 13),
    # angles for pen up/down
    angle_up=10,
    angle_down=0,
)

#bg.set_angles(-120, 0)

# use this to discover the bounds, (min_x, min y, max_x, max_y)
bg.drive_xy()

# use these to test them
#bg.box()
#bg.test_pattern()

# generating
#image_to_json("crow", draw_contours=1.5, repeat_contours=2, draw_hatch=16)
#image_to_json("fish", draw_contours=1.5, repeat_contours=2, draw_hatch=16)
#image_to_json("tree", draw_contours=1.75)
#image_to_json("vhs", draw_contours=2.5)

# ploting
#bg.plot_file("images/crow.json")
