from multi_ball_tracker import FrameProcessor
import math

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
		target_spacing=1, camera_ratio=0.0189634, frame_count=0):
        self.frame = frame
        self.color_range = color_range
        self.servo_angle = servo_angle
        self.target_spacing = target_spacing
        self.camera_ratio = camera_ratio
        self.frame_count = frame_count
        self.frameProcessor = FrameProcessor(frame, frame_count, color_range)
        self.frameProcessor.start()

    """
    Checks to see if the frame processor has finished calculating
    the location of balls in the frame
    """
    def result_available(self):
        return self.frameProcessor.isAlive()

    def get_balls(self):
        return self.frameProcessor.get_balls()

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
        ordered_balls = sorted(balls, key=lambda ball: ball.x)
        angle1 = ratio * math.sqrt(math.pow(balls[0].x - balls[1].x, 2) + math.pow(balls[0].y - balls[1].y, 2))
        angle2 = ratio * math.sqrt(math.pow(balls[1].x - balls[2].x, 2) + math.pow(balls[1].y - balls[2].y, 2))
        num = targetSpacing * sin(angle1+angle2)
        den = (targetSpacing * sin(angle2) / sin(angle1)) - (targetSpacing * cos(angle1+angle2))
        alpha = atan(num/den)
        l = targetSpacing * sin(angle1 + alpha) / sin(angle1)   #length from ball to robot
        x = l * sin(alpha)  #x from right-most ball
        y = -l * cos(alpha)  #y from right-most ball (ball[2]). Add constants if you want the origin as shown in picture
        rotOffset = (balls[1].x - 1640) * ratio
        rotation = servo_angle + rotOffset + alpha + 90
        return(x,y,rotation)

class CameraCapture:
    def __init__(self):
        pass

    def get_current_position(self):
        pass

class Navigation(threading.Thread):

    def __init__(self, autonomy):
        self.autonomy = autonomy
        self.run_flag = True
        self.camera = CameraCapture()

    def run(self):
        while self.run_flag:
            position = camera.get_current_position()
            autonomy.update_current_position(position)


if __name__ == "__main__":
    pass
