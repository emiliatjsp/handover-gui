#!/usr/bin/env python

import rospy
from moveit_commander import MoveGroupCommander
import handover_trajectory_controllers.HandoverTrajectory_interface as htc
from panda_utils_python.panda_utils import panda_utils
from geometry_msgs.msg import WrenchStamped
from franka_msgs.msg import FrankaState
from std_msgs.msg import Float64
from std_msgs.msg import String
import time
import user_studies.object_detection as obj_dec
import typer
import signal
import sys
#from playsound import playsound
from playsound3 import playsound #, AVAILABLE_BACKENDS, DEFAULT_BACKEND
#print(AVAILABLE_BACKENDS) # See available backends on your system
#print(DEFAULT_BACKEND)   # See the default chosen backend
#import pygame 
#from pygame import mixer

# Dependencies for sound
# alsa-utils ffmpeg mpg123 pulseaudio-utils
# ffmpeg -i /home/demo_ws/src/user_studies/audio/sound_files/auto-transparent-approach.mp3 /tmp/test.wav
# aplay -D plughw:1,0 /tmp/test.wav
# Things to put in asoundrc:
    # defaults.pcm.card 1
    # defaults.pcm.device 0
    # defaults.ctl.card 1
# =================================

#amixer set Master 50%

app = typer.Typer(pretty_exceptions_show_locals = False)

default_start_point = [-0.26147651335049266, -0.38403684947300765, 0.05179618217418358]

right_end_point = [0.601126, -0.145253, 0.356328]# [0.601126, 0.245253, 0.356328]
middle_end_point = [0.601126, -0.145253, 0.356328]
left_end_point = [0.601126, -0.445253, 0.356328]

class MsgPublisher():
    def __init__(self):
        self.msg_publisher = rospy.Publisher('/robot_msg', String, queue_size=10)
        self.send_msg(" ")

    def send_msg(self, msg: str):
        self.msg_publisher.publish(msg)

class MovingAverage:
    def __init__(self):
        self.max_size = 5
        self.curr_pos = 0
        self.sum = 0
        self.data = [0] * self.max_size

    def add_entry(self, entry):
        self.sum -= self.data[self.curr_pos]
        self.data[self.curr_pos] = entry
        self.sum += entry
        self.curr_pos = (self.curr_pos +  1) % self.max_size

    def get_average(self):
        return float(self.sum)/self.max_size
    
    def reset(self):
        self.curr_pos = 0
        self.sum = 0
        self.data = [0] * self.max_size

class XYZMovingAverage:
    x_avg = MovingAverage()
    y_avg = MovingAverage()
    z_avg = MovingAverage()
    
    @staticmethod
    def reset():
        XYZMovingAverage.x_avg.reset()
        XYZMovingAverage.y_avg.reset()
        XYZMovingAverage.z_avg.reset()

class HandoverDemo:
    def __init__(self, vision, warning, z_force_thres = -8, x_force_thres = -1):
        self.ext_force_sub = rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self.ext_force_callback)
        self.error_listener = rospy.Subscriber("/franka_state_controller/franka_states/", FrankaState, self.callback_errordetected)
        self.phase_listener = rospy.Subscriber("/phase_publisher", Float64, self.phase_callback)
        self.reached = False
        self.object_released = False
        rospy.set_param('handover_started', False)  
        self.helper = panda_utils()
        self.error_detected = False
        self.moveit_controller = "position_joint_trajectory_controller" #!
        self.vision = vision
        self.warning = warning
        self.commander = MoveGroupCommander('panda_arm')
        self.table_offset = 0.02
        self.z_force_thres = z_force_thres
        self.x_force_thres = x_force_thres
        self.mode = None

    def handover(self, placeholder, pick_success = True):
        active_controller = "ellipse_torque_controller"
        action_type = '/handover_ellipse'
        self.helper.switch_controller(active_controller)
        if self.warning:
            input("press ENTER to start moving ...")
        while not self.reached:
            self.helper.switch_controller(active_controller)
            time.sleep(0.01)    # This is because of how controller_manager works: start -> stop -> start -> update, so if we 
                                # call the action server immediately, we will have racing condition error where motion_finished_
                                # is set to False in ActionCallback, but immediately set to True in stopping().
                
            is_success = htc.handover(placeholder, action_type)
            if not is_success:
                return False

            self.helper.switch_controller('handover_idle_controller')
            self.reached = True
            r = rospy.Rate(60)
            while not rospy.is_shutdown():
                if self.object_released or not pick_success:
                    break
                if not self.reached:
                    rospy.set_param('handover_started', True)
                    break
                r.sleep()
        time.sleep(0.5)
        return True

    def retreat(self):
        self.helper.switch_controller(self.moveit_controller)
        #Go back to home ('ready') position
        self.commander.set_max_velocity_scaling_factor(0.5)
        self.commander.set_named_target('ready')
        is_success = self.commander.go()
        if not is_success:
            while not is_success:
                statement = input("Robot move error. Waiting for command to continue ")
                time.sleep(1)
                self.helper.recovery_client()
                self.commander.set_max_velocity_scaling_factor(0.5)
                self.commander.set_named_target('ready')
                is_success = self.commander.go()
        return True

    def ext_force_callback(self, data):
        if self.object_released == True:
            return
        if self.reached:
            data = data.wrench
            force_x = data.force.x
            force_y = data.force.y 
            force_z = data.force.z
            XYZMovingAverage.x_avg.add_entry(force_x)
            XYZMovingAverage.z_avg.add_entry(force_z)
            z_threshold = self.z_force_thres
            x_threshold = self.x_force_thres
            if self.object_released == False and (XYZMovingAverage.z_avg.get_average() < z_threshold or XYZMovingAverage.x_avg.get_average() < x_threshold):
                print("Release Detected")
                XYZMovingAverage.reset()
                self.object_released = True
                self.helper.move_gripper(width = 0.08)

    def callback_errordetected(self, data):
        if data.robot_mode == 4 or data.robot_mode == 5:
            self.error_detected = True
        return

    def phase_callback(self, data):
        if self.reached:
            if data.data < 0.95:
                self.reached = False

    def cartesian_move(self, x, y, z, theta):
        is_success = htc.cartesian_move(x, y, z, theta)
        if self.error_detected == 1:
            is_success = False
        return is_success

    def go_to_home(self):
        self.reached = False
        self.object_released = False
        rospy.set_param('handover_started', False)

        self.helper.switch_controller(self.moveit_controller)
        
        self.helper.move_gripper(width = 0.08)
        time.sleep(0.1)

        self.commander.set_max_velocity_scaling_factor(0.5)
        self.commander.set_named_target('ready')
        is_success = self.commander.go()
        if not is_success:
            return False

    def object_prep(self, object_name, success=True):
        self.reached = False
        self.object_released = False
        rospy.set_param('handover_started', False)

        self.helper.switch_controller(self.moveit_controller)
        
        self.helper.move_gripper(width = 0.08)
        time.sleep(0.1)

        self.commander.set_max_velocity_scaling_factor(0.5)
        self.commander.set_named_target('ready')
        is_success = self.commander.go()
        if not is_success:
            return False

        if self.vision: 
            self.commander.set_named_target('scanning_high')
            is_success = self.commander.go()
            if not is_success:
                return False

            pose_goal = obj_dec.get_EE_pose_wrt_obj(self.commander, object_name, cartesian = True)
            self.helper.switch_controller('handover_cartesian_controller')
            time.sleep(0.1)
            # Move above object ready for pickup (5cm on top of object)
            is_success = self.cartesian_move(pose_goal[0], pose_goal[1], self.table_offset + pose_goal[2] + 0.05, pose_goal[3])
            time.sleep(0.01)
            # Move along z axis only
            is_success = self.cartesian_move(pose_goal[0], pose_goal[1], self.table_offset + pose_goal[2], 0.0)
            time.sleep(0.01)
            # Close the gripper
            if success:
                is_success = self.helper.close_gripper(width = pose_goal[4], force = 20.0, speed = 0.5) 
                print("Picking up object successfully...")
            else:
                is_success = self.helper.move_gripper(width = pose_goal[6], speed = 0.5) # use error_width
                print("Picking up object unsuccessfully...")
            if not is_success:
                return False
            time.sleep(0.01)
            # Lift the object 20 cm from ground
            is_success = self.cartesian_move(pose_goal[0], pose_goal[1], pose_goal[2] + 0.2, -pose_goal[3])
            time.sleep(0.01) 
        
            self.helper.switch_controller(self.moveit_controller)
            self.commander.set_max_velocity_scaling_factor(0.5)
            self.commander.set_named_target('handover_ready')
            is_success = self.commander.go()
            if not is_success:
                return False
            
        else:
            self.commander.set_named_target('handover_ready')
            is_success = self.commander.go()
            if not is_success:
                return False

            time.sleep(2)
            is_success = self.helper.close_gripper(width = 0.05, force = 20.0, speed = 0.5)
            if not is_success:
                return False
            if success:
                print("Picking up object successfully...")
            else:
                print("Picking up object unsuccessfully...")

        return success

class StudyLoop:
    def __init__(self, obj, transparency):
        self.obj = obj
        self.transparency = transparency
        self.mode = None
        self.gui_listener = rospy.Subscriber("/gui_btn", String, self.gui_callback)
        self.msg_publisher = MsgPublisher()
        self.last_btn = None
        self.detect_fail = False
        self.pickup_fail = False
        self.proceed = False
        self.retry = False
        self.pickup = False
        self.approach = False
        self.retreat = False
        self.object = None
        self.object_in_hand = False
        self.approach_pos = None
        self.home = False
        self.task_running = False
        self.done = False
        self.detect_outcome = [0,1,1,0,1,0,1,1,0,1,1,1,0,1,1] # 25% error
        self.pickup_outcome = [0,1,0,1,1,1,1,0,1,1,1,0,1,1,1] # 25% error

    def signal_handler(self, sig, frame):
        print("\nKeyboardInterrupt detected. Exiting loop gracefully.")
        self.proceed = False
        sys.exit(0)

    def run_loop(self):
        r = rospy.Rate(60)
        self.msg_publisher.send_msg(" ")
        self.done = False
        if self.mode == "autonomous":
            self.autonomous_loop()
        elif self.mode == "semi_autonomous":
            self.semi_autonomous_loop()
        elif self.mode == "manual":
            self.manual_loop()

    def autonomous_loop(self):
        if self.mode != 'autonomous':
            return
        self.obj.helper.recovery_client()
        # TODO: go through list of objects and only pop if success
        #self.go_to_home()

        if self.mode != 'autonomous':
            return
        if self.transparency:
            self.msg_publisher.send_msg("Picking up object...")
            playsound('/home/demo_ws/src/user_studies/audio/sound_files/transparent-pickup.mp3')
            
        pick_success = self.pickup_outcome.pop(0)
        self.pick_up_object(pick_obj="CUBE", success=pick_success)
        self.msg_publisher.send_msg(" ")
        
        if self.mode != 'autonomous':
            return
        detect_success = self.detect_outcome.pop(0)
        if not detect_success:
            time.sleep(7)
            print("ERROR: did not detect hand")
            if self.transparency:
                self.msg_publisher.send_msg("ERROR: did not detect hand")
                playsound('/home/demo_ws/src/user_studies/audio/sound_files/auto-transparent-approach-fail.mp3')
            time.sleep(7)
        
        if self.mode != 'autonomous':
            return
        if self.transparency:
            self.msg_publisher.send_msg("Approaching...")
            playsound('/home/demo_ws/src/user_studies/audio/sound_files/transparent-approaching.mp3')
        self.approach_human(position="middle", pick_success=pick_success)
        self.msg_publisher.send_msg(" ")
    
        #if self.transparency:
        #    mixer.init()
        #    mixer.music.load('/home/demo_ws/src/user_studies/audio/sound_files/transparent-retreating.mp3')
        #    mixer.music.play()
        #self.go_to_home()
        
    def semi_autonomous_loop(self):
        self.obj.helper.recovery_client()
        self.go_to_home()

        print("Ready to pick up object")
        self.msg_publisher.send_msg("Ready to pick up object")
        playsound('/home/demo_ws/src/user_studies/audio/sound_files/semi-all-pickup.mp3')

        signal.signal(signal.SIGINT, self.signal_handler)
        try:
            while not self.proceed:
                if self.done:
                    return
                time.sleep(0.1)
        except Exception as e:
            print(f"\nUnexpected error: {e}")
            raise
        self.proceed = False
        
        # TODO: go through list of objects and only pop if success
        print("Picking up object...")
        if self.transparency:
            self.msg_publisher.send_msg("Picking up object...")
            playsound('/home/demo_ws/src/user_studies/audio/sound_files/transparent-pickup.mp3')
            
        pick_success = self.pickup_outcome.pop(0)
        pickup_successful = self.pick_up_object(pick_obj="CUBE", success=pick_success)
        self.object_in_hand = pickup_successful 
        print('Object in hand: '+str(self.object_in_hand))

        print("Ready to approach...")
        self.msg_publisher.send_msg("Ready to approach...")
        playsound('/home/demo_ws/src/user_studies/audio/sound_files/semi-opaque-approach.mp3')

        signal.signal(signal.SIGINT, self.signal_handler)
        try:
            while not self.proceed:
                if self.done:
                    return
                elif self.retry:
                    if not self.object_in_hand:
                        if self.transparency:
                            self.msg_publisher.send_msg("Picking up object...")
                            playsound('/home/demo_ws/src/user_studies/audio/sound_files/transparent-pickup.mp3')
                        pick_success = True
                        pickup_success = self.pick_up_object(pick_obj="CUBE", success=pick_success)
                        self.object_in_hand = pickup_success 
                        print('Object in hand: '+str(self.object_in_hand))
                        self.retry = False
                        print("Ready to approach...")
                        self.msg_publisher.send_msg("Ready to approach...")
                        playsound('/home/demo_ws/src/user_studies/audio/sound_files/semi-opaque-approach.mp3')
                    else:
                        print("ERROR: cannot pick up object if still carrying one")
                        self.retry = False
                        if self.transparency:
                            self.msg_publisher.send_msg("ERROR: cannot pick up object if still carrying one")
                        
                time.sleep(0.1)
        except Exception as e:
            print(f"\nUnexpected error: {e}")
            raise
        self.proceed = False

        print("Waiting until hand is visible...")
        detect_success = self.detect_outcome.pop(0)
        if not detect_success:
            time.sleep(7)
            print("ERROR: did not detect hand")
            if self.transparency:
                self.msg_publisher.send_msg("ERROR: did not detect hand")
                playsound('/home/demo_ws/src/user_studies/audio/sound_files/auto-transparent-approach-fail.mp3')
            time.sleep(7)
        self.proceed = False

        print("Approaching...")
        if self.transparency:
            self.msg_publisher.send_msg("Approaching...")
            playsound('/home/demo_ws/src/user_studies/audio/sound_files/transparent-approaching.mp3')
        self.approach_human(position="middle", pick_success=pick_success)
        self.object_in_hand = False

        print("Ready to retreat...")
        self.msg_publisher.send_msg("Ready to retreat...")
        playsound('/home/demo_ws/src/user_studies/audio/sound_files/semi-all-retreat.mp3')
        signal.signal(signal.SIGINT, self.signal_handler)
        try:
            while not self.proceed:
                if self.done:
                    return
                time.sleep(0.1)
        except Exception as e:
            print(f"\nUnexpected error: {e}")
            raise
        self.proceed = False
        self.robot_retreat()

    def manual_loop(self):
        self.obj.helper.recovery_client()
        if not self.task_running:
            self.go_to_home()
            self.home = True # avoid approaching from here
            self.task_running = True

        elif self.pickup and self.object!=None:
            if self.object_in_hand:
                print("ERROR: cannot pick up object if still carrying one")
                if self.transparency:
                    self.msg_publisher.send_msg("ERROR: cannot pick up object if still carrying one")
                self.pickup = False
                self.object = None
            else:
                print("Picking up object...")
                if self.transparency:
                    self.msg_publisher.send_msg("Picking up object...")
                    playsound('/home/demo_ws/src/user_studies/audio/sound_files/transparent-pickup.mp3')

                pick_success = self.pickup_outcome.pop(0)
                pickup_successful = self.pick_up_object(pick_obj="CUBE", success=pick_success)
                self.object_in_hand = pickup_successful 
                print('Object in hand: '+str(self.object_in_hand))
                self.pickup = False
                self.object = None
                self.home = False
                self.msg_publisher.send_msg(" ") 
                # TODO: go through list of objects and only pop if success
        
        elif self.approach and self.approach_pos!=None and not self.home:
            print("Approaching...")
            if self.transparency:
                self.msg_publisher.send_msg("Approaching...")
                playsound('/home/demo_ws/src/user_studies/audio/sound_files/transparent-approaching.mp3')

            self.approach_human(position=self.approach_pos)
            self.approach = False
            self.approach_pos = None
            self.home = True # avoid approaching from here
            self.object_in_hand = False
            self.msg_publisher.send_msg(" ")

        elif self.retreat:
            print("Retreating...")
            if self.transparency:
                self.msg_publisher.send_msg("Retreating...")
                #playsound('/home/demo_ws/src/user_studies/audio/sound_files/transparent-retreat.mp3')
            self.robot_retreat()
            self.retreat = False
            self.msg_publisher.send_msg(" ")
                
            #self.objects.pop() # del
            # TODO: CHECK FOR OBJECT IN HAND (F > T)
            #self.object_in_hand = False

    def go_to_home(self):
        self.obj.go_to_home()
    
    def pick_up_object(self, pick_obj="CUBE", success=True):
        outcome = self.obj.object_prep(pick_obj, success=success)
        return outcome
    
    def approach_human(self, position = "middle", pick_success = True):
        current_pose = self.obj.commander.get_current_pose(end_effector_link = "panda_hand_tcp")
        
        if position == "middle":
            print("set end point to middle")
            rospy.set_param('final_x', middle_end_point[0])
            rospy.set_param('final_y', middle_end_point[1])
            rospy.set_param('final_z', middle_end_point[2])
        elif position == "left":
            print("set end point to left")
            rospy.set_param('final_x', left_end_point[0])
            rospy.set_param('final_y', left_end_point[1])
            rospy.set_param('final_z', left_end_point[2])
        elif position == "right":
            print("set end point to right")
            rospy.set_param('final_x', right_end_point[0])
            rospy.set_param('final_y', right_end_point[1])
            rospy.set_param('final_z', right_end_point[2])

        rospy.set_param('init_x', current_pose.pose.position.x)
        rospy.set_param('init_y', current_pose.pose.position.y)
        rospy.set_param('init_z', current_pose.pose.position.z)
        rospy.set_param('traj_time', 4.0)

        self.obj.handover(int(0), pick_success)
        self.obj.helper.stop_gripper()
    
    def robot_retreat(self):
        self.obj.retreat()

    def gui_callback(self, data):
        self.last_btn = data.data
        if data.data in ["autonomous","semi_autonomous","manual"]:
            self.mode = data.data
            self.obj.mode = self.mode
            print("mode: "+self.mode)
        elif data.data == "detect_fail":
            self.detect_fail = True
        elif data.data == "pickup_fail":
            self.pickup_fail = True
        elif data.data == "proceed":
            self.proceed = True
        elif data.data == "pickup":
            self.pickup = True
        elif data.data == "approach":
            self.approach = True
        elif data.data == "retreat":
            self.retreat = True
        elif data.data in ["1","2","3","4","5"]:
            self.object = data.data
        elif data.data in ["left","middle","right"]:
            self.approach_pos = data.data
        elif data.data == 'retry':
            self.retry = True
        elif data.data == 'confirm':
            self.task_running = False
            self.mode = None
            self.done = True
            self.reset()
    
    def reset(self):
        self.object_in_hand = False

@app.command()
def main(vision: bool = typer.Option(default = True, help = "Use the Realsense or not"),
        warning: bool = typer.Option(default = True, help = "Warn before moving or not"),
        transparency: bool = typer.Option(default = True, help = "Transparent robot or not")):

    rospy.init_node('trust_study')
    run_obj = HandoverDemo(vision = vision, warning = warning)
    study_loop = StudyLoop(run_obj, transparency = transparency)

    while not rospy.is_shutdown():
        study_loop.run_loop()

if __name__ == "__main__":
    app()
