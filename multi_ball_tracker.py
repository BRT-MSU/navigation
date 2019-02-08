# USAGE
# python ball_tracking.py --video ball_tracking_example.mp4
# python ball_tracking.py

# import the necessary packages
from collections import deque
from copy import deepcopy
import numpy as np
import argparse
import imutils
import cv2
import threading
from timeit import default_timer as timer

class Ball:

    def __init__(self,
            circle_center, moment_center, radius, color, hsv):
        self.circle_center = circle_center
	self.moment_center = moment_center
	self.radius = radius
	self.color = color
	self.hsv = hsv
        self.x = circle_center[0]

	"""
    Get the center of the ball that is identified by the hsv values.
    Note: This uses cv2.minEnclosingCircle(). This result can be
    different by several pixels from the moment center. The difference
    between these two methods is documented at https://docs.opencv.org/3.1.0/d3/dc0/group__imgproc__shape.html#ga8ce13c24081bbc7151e9326f412190f1.
    """
    def get_circle_center(self):
        return self.circle_center
	"""
    Get the center of the ball that is identified by the hsv values.
    Note: This uses cv2.moments(c) to calculate the center pixel of
    the ball. This will always an integer pair.
    The difference between these two methods is documented at https://docs.opencv.org/3.1.0/d3/dc0/group__imgproc__shape.html#ga8ce13c24081bbc7151e9326f412190f1.
    """
    def get_moment_center(self):
        return self.moment_center

    def get_radius(self):
        return self.radius

    def get_color(self):
        return self.color

    def get_hsv(self):
        return self.hsv
    def __eq__(self, other):
            return (self.get_circle_center() == other.get_circle_center() and
                    self.get_radius() == other.get_radius() and
                    self.get_hsv() == other.get_hsv())

    def __lt__(self, other):
        return self.x < other.x

    def __gt__(self, other):
        return self.x > other.x

    def __str__(self):
        return str(self.color) + " : " + str(self.get_circle_center())

    def __repr_(self):
        return repr((self.color, self.get_circle_center()))

class ColorProcessor (threading.Thread):
    def __init__(self, hsv, frame, color, color_bounds):
        threading.Thread.__init__(self)
        self.hsv = hsv
        self.frame = frame
        self.color = color
        self.color_bounds = color_bounds
        self.circle_center = (-1, -1)
        self.moment_center = None
        self.radius_circle = 0

    def run(self):
        mask = cv2.inRange(self.hsv, self.color_bounds['lower'], self.color_bounds['upper'])
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
	if len(cnts) > 0:
	    c = max(cnts, key=cv2.contourArea)
	    (self.circle_center, self.radius_circle) = cv2.minEnclosingCircle(c)
	    M = cv2.moments(c)
	    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	    if self.radius_circle > 10:
                self.moment_center = center


    def join(self):
        threading.Thread.join(self)
	ball = Ball(self.circle_center, self.moment_center, self.radius_circle,
		self.color, self.color_bounds)
	return ball

class FrameProcessor(threading.Thread):

    def __init__(self, frame, frame_count, color_range, display=False):
        threading.Thread.__init__(self)
        self.frame = frame
        self.frame_count = frame_count
        self.color_range = color_range
        self.threads = []
        self.balls = {}
        self.display = display

    def run(self):
        # resize the frame
        #frame = imutils.resize(frame, width=600)

        # convert the frame to HSV
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        # process the frame for every different ball
        for color in color_range:
            processor = ColorProcessor(hsv, self.frame, color, self.color_range[color])
            self.threads.append(processor)
            processor.start()
        display = args.get("display")
        for thread in self.threads:
            ball = thread.join()
            if ball.get_radius() > 10:
                self.balls[ball.get_color()] = ball
                if self.display:
                    (x, y) = ball.get_circle_center()
                    cv2.putText(self.frame, ball.get_color(),
			    (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX,
			    1, (0, 255, 0), 2)
                    cv2.circle(self.frame, (int(x), int(y)), int(ball.get_radius()),
			    (0, 255, 255), 2)
                    cv2.circle(self.frame, ball.get_moment_center(), 5,
                            (0, 0, 255), -1)
        if self.display:
            cv2.putText(self.frame, str("frame: " + str(self.frame_count)),
		    (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Frame", self.frame)

    def get_balls(self):
        if len(self.balls) > 0:
            return self.balls
        return None

def process_video(color_range, camera):
    i = 0
    total_time = 0
    try:
        while True:
            i += 1
            # grab the current frame
            (grabbed, frame) = camera.read()
            # then we have reached the end of the video
            if args.get("video") and not grabbed:
                break
            key = cv2.waitKey(1) & 0xFF
            # if the 'q' key is pressed, stop the loop
            if key == ord("q"):
                break
            beginning_time = timer()
            frame_processor = FrameProcessor(frame, i, color_range,
                    args.get("display"))
            frame_processor.start()
            frame_processor.join()
            ending_time = timer()
	    total_time += (ending_time - beginning_time)
            balls = frame_processor.get_balls()
    except(KeyboardInterrupt):
        pass
    print("average frame time:", (total_time/i))
def main (color_range, camera):
    process_video(color_range, camera)

if __name__ == "__main__":
    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()

    ap.add_argument("-d", "--display", type=bool, default=False,
            help="display to hdmi?")
    ap.add_argument("-v", "--video",
            help="path to the (optional) video file")
    ap.add_argument("-b", "--buffer", type=int, default=10,
            help="max buffer size")
    args = vars(ap.parse_args())

    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    color_range = {
        'orange' : { 'lower': (5, 112, 93), 'upper': (70, 255, 255)},
        'green': {'lower': (47, 41, 46), 'upper': (80, 155, 255)},
        'blue': {'lower': (93, 139, 89), 'upper': (126, 255, 255)}
    }

    # if a video path was not supplied, grab the reference
    # to the webcam
    if not args.get("video", False):
        camera = cv2.VideoCapture(0)

    # otherwise, grab a reference to the video file
    else:
        camera = cv2.VideoCapture(args["video"])
    # cleanup the camera and close any open windows
    main(color_range, camera)
    camera.release()
    cv2.destroyAllWindows()

