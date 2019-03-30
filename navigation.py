from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from multi_ball_tracker import FrameProcessor
import math
import threading
import cv2

class Frame:
    """
    Starts the calculations done to locate the balls in the frame.
    :param frame: numpy array that is provided from cv2:VideoCapture:read()
    :param color_range: a dictionary in the form of
        {color_identifier : (hsv_min, hsv_max) } as calculated from
        range-detector.py
    :param frame_count: optional interger used to describe the frame number
    """
    def __init__(self, frame, color_range, servo_angle=0,
		target_spacing=82.5, camera_ratio=0.0189634, frame_count=0):
        self.frame = frame
        self.color_range = color_range
        self.servo_angle = servo_angle
        self.target_spacing = target_spacing
        self.camera_ratio = camera_ratio
        self.frame_count = frame_count
        self.frame_processor = FrameProcessor(frame, frame_count, color_range)
        self.frame_processor.start()
        print("frame: " +str(frame_count))
    """
    Checks to see if the frame processor has finished calculating
    the location of balls in the frame
    """
    def result_available(self):
        return self.frame_processor.isAlive()

    def join_frame(self):
        self.frame_processor.join()

    def get_balls(self):
        return self.frame_processor.get_balls()

    """
    Calculate the position of the robot using the center point coordinates of the balls/targets.
    Ball1 is the origin
	    ----------------------------
	    |			      		   |      
	    |    A    Starting     B   |                    90
	    |                          |                     ^
   	BLUE|Ball1a              Ball1b|RED         180 < rotation > 0
   GREEN|Ball2a              Ball2b|GREEN                v
 	 RED|Ball3a              Ball3b|BLUE                270
   	    |                          |
 	    |                          |
 	   ^|                          |
  	   ||          Mining          |
	   y----------------------------
      (0,0) x->
    """
    def calculate_xyr(self, balls):
        if balls is None:
            return (-1, -1, -1)
        ordered_balls = sorted(balls, key=lambda ball: ball.x)
        angle1, angle2 = self.calcAngles(balls)
        num = self.targetSpacing * math.sin(angle1+angle2)
        den = (self.targetSpacing * math.sin(angle2) / math.sin(angle1)) - (self.targetSpacing * math.cos(angle1+angle2))
        alpha = math.atan(num/den)
        l = self.targetSpacing * math.sin(angle1 + alpha) / math.sin(angle1)   #length from ball to robot
        x = l * math.sin(alpha)  #x from right-most ball
        y = -l * math.cos(alpha)  #y from right-most ball (ball[2]). Add constants if you want the origin as shown in picture
        # picturerotOffset = (balls[1].x - 1640) * ratio
        # rotation = servo_angle + rotOffset + alpha + 90
        return (x, y)

    def calcAngles(self, balls):
        HFOV = 62.2 / 2
        ordered_balls = sorted(balls, key=lambda ball: ball.x)
        angle1 = 0
        angle2 = 0
        relativePos = []
        tempAngles = []
        for ball in ordered_balls:
            relativePos.append(abs(ball.x-540))

        for ball in relativePos:
            tempAngles.append(math.atan((2*ball*math.tan(math.radians(HFOV)))/1080.))

        #Balls on same side of center
        if(ordered_balls[0].x <= 540 and ordered_balls[2]<=540) or (ordered_balls[0].x >= 540 and ordered_balls[2]>=540):
            if ordered_balls[0].x > 540:
                angle1 = tempAngles[1]-tempAngles[0]
                angle2 = tempAngles[2]-angle1
            else:
                angle2 = tempAngles[2]-tempAngles[1]
                angle1 = tempAngles[0]-angle1
        elif(ordered_balls[0].x < 540 and ordered_balls[1] <=540):
            angle1 = tempAngles[0]+tempAngles[1]
            angle2 = tempAngles[2]
        else:
            angle1 = tempAngles[0]
            angle2 = tempAngles[1] + tempAngles[2]

        return angle1, angle2

class CameraCapture(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

        # initialize the camera and grab a reference to the raw camera capture
        self.camera = PiCamera(sensor_mode=2)
        self.camera.resolution = (1920, 1080)
        self.camera.framerate = 10
        self.raw_capture = PiRGBArray(self.camera, size=(1920, 1080))
        self.color_range = {
            'orange': { 'lower': (5, 112, 93), 'upper': (70, 255, 255)},
            'green': {'lower': (47, 41, 46), 'upper': (80, 155, 255)},
            'blue': {'lower': (93, 139, 89), 'upper': (126, 255, 255)}
        }
        print(self.color_range)
        self.frame = None
        # allow the camera to warmup
        time.sleep(0.1)

    def run(self):
        # capture frames from the camera
        for frame in self.camera.capture_continuous(self.raw_capture, format="bgr",
                use_video_port=True):
            # grab the raw NumPy array representing the image, 
            # then initialize the timestamp
            # and occupied/unoccupied text
            image = frame.array
            frame_object = Frame(image, self.color_range)
            # show the frame
            cv2.imshow("Frame", image)
            key = cv2.waitKey(1) & 0xFF

            frame_object.join_frame()
            self.frame = frame_object
            # clear the stream in preparation for the next frame
            self.raw_capture.truncate(0)

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                break       



    def get_current_position(self):
        if self.frame is not None:
            balls = self.frame.get_balls()
            return self.frame.calculate_xyr(balls)
        return None                

class Navigation(threading.Thread):

    def __init__(self, autonomy):
        threading.Thread.__init__(self)
#        self.autonomy = autonomy
        self.run_flag = True
        self.camera = None

    def run(self):
        self.camera = CameraCapture()
        self.camera.start()
        while self.run_flag:
            position = self.camera.get_current_position()
            print(position)
   #         autonomy.update_current_position(position)


if __name__ == "__main__":
    navigation = Navigation(None)
    navigation.start()

