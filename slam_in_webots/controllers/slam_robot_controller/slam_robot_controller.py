"""slam_robot_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Motor
from controller import Camera
import rospy
from sensor_msgs.msg import Image, CameraInfo
import math
import numpy as np

def set_velocity(cmd_vel): # cmd_vel=[vx(m/s),omega(rad/s)]
    motors[0].setVelocity(cmd_vel[0]*20-cmd_vel[1]*2.8)
    motors[1].setVelocity(cmd_vel[0]*20+cmd_vel[1]*2.8)

def publish_camera(i):   # publish image to ROS topic
    img_msg = Image()
    img_msg.header.stamp = rospy.Time.from_seconds(time)
    # img_msg.header.frame_id = camera_optical_frame
    img_msg.height = cameras[i].getHeight()
    img_msg.width = cameras[i].getWidth()
    img_msg.encoding = "bgra8"
    img_msg.step = 4 * cameras[i].getWidth()
    img = cameras[i].getImage()
    img_msg.data = img
    if(i==0):
        pub_l_cam.publish(img_msg)
    elif(i==1):
        pub_r_cam.publish(img_msg)
    
def publish_camera_info():  # publish camera info to ROS topic
    cam_info = CameraInfo()
    cam_info.header.stamp = rospy.Time.from_seconds(time)
    cam_info.height = cameras[0].getHeight()
    cam_info.width = cameras[0].getWidth()
    f_y = mat_from_fov_and_resolution(
        h_fov_to_v_fov(cameras[0].getFov(), cam_info.height, cam_info.width),
        cam_info.height)
    f_x = mat_from_fov_and_resolution(cameras[0].getFov(), cam_info.width)
    cam_info.K = [f_x, 0, cam_info.width / 2,
                       0, f_y, cam_info.height / 2,
                       0, 0, 1]
    cam_info.P = [f_x, 0, cam_info.width / 2, 0,
                       0, f_y, cam_info.height / 2, 0,
                       0, 0, 1, 0]
    cam_info.header.frame_id = "map"  #need to be modified
    pub_l_cam_info.publish(cam_info)
    pub_r_cam_info.publish(cam_info)

def mat_from_fov_and_resolution(fov, res):
    return 0.5 * res * (math.cos((fov / 2)) / math.sin((fov / 2)))
    
def h_fov_to_v_fov(h_fov, height, width):
    return 2 * math.atan(math.tan(h_fov * 0.5) * (height / width))

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# initialization
cameras = []
cameras.append(robot.getDevice("left_camera"))
cameras.append(robot.getDevice("right_camera"))
for i in range(2):
    cameras[i].enable(timestep)

motors = []
motors.append(robot.getDevice('left_motor'))
motors.append(robot.getDevice('right_motor'))
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)


# ros_initialization
rospy.init_node("webots_ros_interface")
pub_l_cam = rospy.Publisher("/stereo/left/image_raw", Image, queue_size=1)
pub_r_cam = rospy.Publisher("/stereo/right/image_raw", Image, queue_size=1)
pub_l_cam_info = rospy.Publisher("/stereo/left/camera_info", CameraInfo, queue_size=1, latch=True)
pub_r_cam_info = rospy.Publisher("/stereo/right/camera_info", CameraInfo, queue_size=1, latch=True)


cmd_vel = [0.0,0.2]  # [vx(m/s),omega(rad/s)]
set_velocity(cmd_vel)

time=0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:

    for i in range(2):
       publish_camera(i)
    publish_camera_info()

    time += timestep / 1000
    pass

# Enter here exit cleanup code.


