#!/usr/bin/env python
from topic_tools.srv import MuxAdd, MuxAddRequest, MuxSelect, MuxSelectRequest
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
sys.path.append('/home/demo_ws/src/user_studies')
from handover_gui.monash_gui.gui_app.csv_functions import CSV
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

right_end_point = [0.601126, 0.245253, 0.356328] #[0.601126, -0.145253, 0.356328]#
middle_end_point = [0.601126, -0.145253, 0.356328]
left_end_point = [0.601126, -0.445253, 0.356328] # TODO

class PhasePublisherMux:
    def __init__(self):
        rospy.wait_for_service('mux_phase/add')
        rospy.wait_for_service('mux_phase/select')
        self.mux_add = rospy.ServiceProxy('mux_phase/add', MuxAdd)
        self.mux_select = rospy.ServiceProxy('mux_phase/select', MuxSelect)
        #self.add_input_topic('phase_publisher')
        #self.add_input_topic('phase_publisher_t')
        self.select_input('phase_publisher')
    
    def add_input_topic(self, topic_name):
        request = MuxAddRequest()
        request.topic = topic_name
        try:
            response = self.mux_add(request)
            rospy.loginfo(f"Added input topic: {topic_name}")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to add topic {topic_name}: {e}")
            return False
    
    def select_input(self, topic_name):
        request = MuxSelectRequest()
        request.topic = topic_name
        try:
            response = self.mux_select(request)
            rospy.loginfo(f"Selected topic: {topic_name}")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to select topic {topic_name}: {e}")
            return False

class MsgPublisher():
    def __init__(self):
        self.msg_publisher = rospy.Publisher('/robot_msg', String, queue_size=10)
        self.send_msg("\n")

    def send_msg(self, msg: str):
        self.msg_publisher.publish(msg)

class PhasePublisherT():
    def __init__(self):
        self.phase_publisher_t = rospy.Publisher('/phase_publisher_t', Float64, queue_size=10)

    def send_msg(self, msg: float):
        self.phase_publisher_t.publish(1.0)

class GuiBlocker():
    def __init__(self):
        self.gui_blocker = rospy.Publisher('/gui_blocker', String, queue_size=10)
        self.gui_blocker.publish("enable")

    def enable(self):
        self.gui_blocker.publish("enable")
    
    def disable(self):
        self.gui_blocker.publish("disable")

class ModeBlocker():
    def __init__(self):
        self.mode_blocker = rospy.Publisher('/mode_blocker', String, queue_size=10)
        self.mode_blocker.publish("enable")

    def enable(self):
        self.mode_blocker.publish("enable")
    
    def disable(self):
        self.mode_blocker.publish("disable")

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
    def __init__(self, vision, warning, transparency, z_force_thres = -8, x_force_thres = -1):
        self.ext_force_sub = rospy.Subscriber("/franka_state_controller/F_ext", WrenchStamped, self.ext_force_callback)
        self.error_listener = rospy.Subscriber("/franka_state_controller/franka_states/", FrankaState, self.callback_errordetected)
        self.phase_listener = rospy.Subscriber("/phase_publisher_mux", Float64, self.phase_callback)
        self.phase_publisher_t = PhasePublisherT()
        self.reached = False
        self.object_released = False
        rospy.set_param('handover_started', False)  
        self.helper = panda_utils()
        self.error_detected = False
        self.moveit_controller = "position_joint_trajectory_controller" #!
        self.vision = vision
        self.warning = warning
        self.transparency = transparency
        self.commander = MoveGroupCommander('panda_arm')
        self.table_offset = 0.02
        self.z_force_thres = z_force_thres
        self.x_force_thres = x_force_thres
        self.mode = None
        self.counter = 0
        self.no_hand = True
        self.active = False
        self.slow_speed = 0.15
        self.medium_speed = 0.45
        self.fast_speed = 0.9
        self.csv = CSV(path_to_file='/home/demo_ws/src/user_studies/handover_gui/monash_gui/')

    def handover(self, placeholder, pick_success = True):
        self.active = True
        self.no_hand = True
        active_controller = "ellipse_torque_controller"
        action_type = '/handover_ellipse'
        self.helper.switch_controller(active_controller)
        if self.warning:
            input("press ENTER to start moving ...")
        
        self.counter = 0
        self.csv.update_csv(None,None,None,None,'r_handover_start')
        while not self.reached:
            self.helper.switch_controller(active_controller)
            time.sleep(0.01)    # This is because of how controller_manager works: start -> stop -> start -> update, so if we 
                                # call the action server immediately, we will have racing condition error where motion_finished_
                                # is set to False in ActionCallback, but immediately set to True in stopping().
            
            r = rospy.Rate(60)
            while self.no_hand:
                if self.mode == 'manual':
                    self.phase_publisher_t.send_msg(1.0)
                time.sleep(0.5)
            is_success = htc.handover(placeholder, action_type)
            if not is_success:
                self.active = False
                return False

            self.helper.switch_controller('handover_idle_controller')
            time.sleep(0.01)
            self.reached = True
            
            while not rospy.is_shutdown():
                if self.object_released or not pick_success:
                    self.csv.update_csv(None,None,None,None,'r_release_object')
                    break
                if not self.reached:
                    rospy.set_param('handover_started', True)
                    break
                r.sleep()
        time.sleep(0.5)
        self.active = False
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
        if self.active:
            self.counter += 1
            if data.data < 0.70 and self.counter > 100: #!
                print("ERROR: did not detect hand")
                if self.transparency:
                    playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/detect_error.mp3')
                print("Did not detect hand with distance {d} and counter {c}".format(d=data.data,c=self.counter))
                self.counter = 0
            elif data.data >= 0.70:
                print("Detected hand with distance {d} and counter {c}".format(d=data.data,c=self.counter))
                self.no_hand = False
                self.counter = 0
            if self.reached:
                print("Reached hand with distance {d} and counter {c}".format(d=data.data,c=self.counter))
                if data.data < 0.95:
                    self.reached = False
                self.counter = 0

    def cartesian_move(self, x, y, z, theta):
        is_success = htc.cartesian_move(x, y, z, theta)
        if self.error_detected == 1:
            is_success = False
        return is_success

    def go_to_home(self, speed='medium'):
        self.csv.update_csv(None,None,None,None,'r_go_to_home')
        self.reached = False
        self.object_released = False
        rospy.set_param('handover_started', False)

        self.helper.switch_controller(self.moveit_controller)
        
        self.helper.move_gripper(width = 0.08)
        time.sleep(0.1)

        if speed == 'slow':
            self.commander.set_max_velocity_scaling_factor(self.slow_speed)
        elif speed == 'medium':
            self.commander.set_max_velocity_scaling_factor(self.medium_speed)
        elif speed == 'fast':
            self.commander.set_max_velocity_scaling_factor(self.fast_speed)
        self.commander.set_named_target('ready')
        is_success = self.commander.go()
        if not is_success:
            return False

    def retreat(self, speed='medium'):
        self.csv.update_csv(None,None,None,None,'r_retreat')
        self.reached = False
        self.object_released = False
        rospy.set_param('handover_started', False)
        self.helper.switch_controller(self.moveit_controller)
        time.sleep(0.1)
        # Go back to home ('ready') position
        if speed == 'slow':
            self.commander.set_max_velocity_scaling_factor(self.slow_speed)
        elif speed == 'medium':
            self.commander.set_max_velocity_scaling_factor(self.medium_speed)
        elif speed == 'fast':
            self.commander.set_max_velocity_scaling_factor(self.fast_speed)
        self.commander.set_named_target('ready')
        is_success = self.commander.go()
        if not is_success:
            while not is_success:
                statement = input("Robot move error. Waiting for command to continue ")
                time.sleep(1)
                self.helper.recovery_client()
                if speed == 'slow':
                    self.commander.set_max_velocity_scaling_factor(self.slow_speed)
                elif speed == 'medium':
                    self.commander.set_max_velocity_scaling_factor(self.medium_speed)
                elif speed == 'fast':
                    self.commander.set_max_velocity_scaling_factor(self.fast_speed)
                self.commander.set_named_target('ready')
                is_success = self.commander.go()
        return True

    def object_prep(self, object_name, success=True, speed='medium'):
        self.reached = False
        self.object_released = False
        rospy.set_param('handover_started', False)

        self.helper.switch_controller(self.moveit_controller) #!!!
        
        self.helper.move_gripper(width = 0.08)
        time.sleep(0.1)

        print('pickup with speed: '+speed)
        if speed == 'slow':
            self.commander.set_max_velocity_scaling_factor(self.slow_speed)
        elif speed == 'medium':
            self.commander.set_max_velocity_scaling_factor(self.medium_speed)
        elif speed == 'fast':
            self.commander.set_max_velocity_scaling_factor(self.fast_speed)
        #self.commander.set_named_target('ready')
        #is_success = self.commander.go()
        #if not is_success:
        #    return False

        if self.vision: 
            self.commander.set_named_target('scanning_high')
            is_success = self.commander.go()
            if not is_success:
                return False

            # TODO: check if object in workspace else return
            pose_goal = obj_dec.get_EE_pose_wrt_obj(self.commander, object_name, cartesian = True)
            if pose_goal:
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
                    #is_success = self.helper.close_gripper(width = pose_goal[4], force = 20.0, speed = 0.5)  #!!!!!
                    is_success = self.helper.close_gripper(width = pose_goal[4], force = 10.0, speed = 0.5)  # changed force for testing
                    print('close gripper')
                    self.csv.update_csv(None,None,None,None,'r_pickup_success: '+object_name)
                    print("Picking up object successfully...")
                else:
                    is_success = self.helper.move_gripper(width = pose_goal[6], speed = 0.5) # use error_width
                    self.csv.update_csv(None,None,None,None,'r_pickup_failure: '+object_name)
                    print("Picking up object unsuccessfully...")
                if not is_success:
                    return False
                time.sleep(0.01)
                # Lift the object 20 cm from ground
                is_success = self.cartesian_move(pose_goal[0], pose_goal[1], pose_goal[2] + 0.2, -pose_goal[3])
                time.sleep(0.01) 
            
            #elif pose_goal is None:
            #    return None
        
            self.helper.switch_controller(self.moveit_controller)
            if speed == 'slow':
                self.commander.set_max_velocity_scaling_factor(self.slow_speed)
            elif speed == 'medium':
                self.commander.set_max_velocity_scaling_factor(self.medium_speed)
            elif speed == 'fast':
                self.commander.set_max_velocity_scaling_factor(self.fast_speed)

            self.commander.set_named_target('handover_ready_high')
            is_success = self.commander.go()
            
            if not is_success:
                return False
            if pose_goal is None:
                return None
            if not pose_goal:
                return False
            
        else:
            self.commander.set_named_target('handover_ready_high')
            is_success = self.commander.go()
            if not is_success:
                return False

            time.sleep(2)
            is_success = self.helper.close_gripper(width = 0.05, force = 20.0, speed = 0.5)
            if not is_success:
                return False
            if success:
                print("Picking up object successfully...")
                self.csv.update_csv(None,None,None,None,'r_pickup_success: '+object_name)
            else:
                print("Picking up object unsuccessfully...")
                self.csv.update_csv(None,None,None,None,'r_pickup_failure: '+object_name)

        return success

class StudyLoop:
    def __init__(self, obj, transparency, training):
        # settings
        self.obj = obj
        self.transparency = transparency
        self.training = training
        self.mode = None
        if self.training:
            self.detect_outcome = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1] 
            self.pickup_outcome = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1] 
        else:
            self.detect_outcome = [1,0,1,1,1,0,1,1,0,1,1,1,0,1,1,1,1,1,1,1,1,1] #[0,1,1,0,1,1,1,0,1,1,1,1,1,1,1] # # 25% error
            self.pickup_outcome = [1,1,1,0,1,1,1,0,0,1,1,1,1,0,1,1,1,1,1,1,1,1] #[1,0,0,1,1,1,1,0,1,1,1,1,1,1,1] # # 25% error
        self.object_list = ['SUGAR','DOCK','SPEAKER','COFFEE','TEA']
        self.failed_object_list = []
        # ros
        self.gui_listener = rospy.Subscriber("/gui_btn", String, self.gui_callback)
        self.msg_publisher = MsgPublisher()
        self.gui_blocker = GuiBlocker()
        self.mode_blocker = ModeBlocker()
        self.mux = PhasePublisherMux()
        # btns
        self.last_btn = None
        self.proceed = False
        self.retry = False
        self.pickup = False
        self.approach = False
        self.retreat = False
        self.object = None
        self.approach_pos = None
        # task vars        
        self.object_in_hand = False
        self.pose = None
        self.task_running = False
        self.done = False
        self.speed = None
        self.gui_blocker.enable()
        self.csv = CSV(path_to_file='/home/demo_ws/src/user_studies/handover_gui/monash_gui/')
        
    def signal_handler(self, sig, frame):
        print("\nKeyboardInterrupt detected. Exiting loop gracefully.")
        self.reset()
        self.proceed = False
        sys.exit(0)

    def run_loop(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        try:
            if len(self.object_list) == 0:
                self.gui_blocker.enable()
            r = rospy.Rate(60)
            self.msg_publisher.send_msg("\n")
            self.done = False #!
            #has_pending_objects = len(self.object_list) != 0 or len(self.failed_object_list) != 0
            if self.mode == "autonomous" and len(self.object_list) != 0:
                self.autonomous_loop()
            elif self.mode == "semi_autonomous": #and has_pending_objects:
                self.semi_autonomous_loop()
            elif self.mode == "manual":
                self.manual_loop()
        except Exception as e:
            print(f"\nUnexpected error: {e}")
            raise

    def autonomous_loop(self):
        self.gui_blocker.enable()
        self.mode_blocker.enable()
        self.obj.helper.recovery_client()
        if not self.task_running:
            self.obj.go_to_home()
            self.task_running = True
        
        self.voice_handler('pickup')
        pick_success = self.pickup_outcome.pop(0)
        self.pick_up_object(pick_obj=self.object_list[0], success=pick_success)
        
        self.voice_handler('')
        detect_success = self.detect_outcome.pop(0)
        if not detect_success:
            time.sleep(7)
            self.csv.update_csv(None,None,None,None,'r_detect_error')
            self.voice_handler('detect_error')
            time.sleep(7)
            self.voice_handler()
        
        self.voice_handler('approach')
        self.approach_human(position="middle", pick_success=pick_success)    
        self.voice_handler('retreat')
        self.obj.go_to_home()
        if len(self.object_list)==0:
            self.voice_handler('done')

        if self.mode != 'autonomous':
            self.task_running = False
            self.gui_blocker.enable()
            return
        
    def semi_autonomous_loop(self):
        self.mode_blocker.enable()
        self.obj.helper.recovery_client()
        if not self.task_running:
            self.gui_blocker.disable()
            self.obj.go_to_home()
            self.task_running = True
        self.voice_handler('pickup_ready')
        self.gui_blocker.enable()

        signal.signal(signal.SIGINT, self.signal_handler)
        try:
            while not self.proceed:
                if self.done:
                    self.gui_blocker.enable()
                    return
                elif self.retry:
                    self.voice_handler('retry_error')
                    self.retry = False
                    self.gui_blocker.enable()
                time.sleep(0.1)
        except Exception as e:
            print(f"\nUnexpected error: {e}")
            raise

        self.proceed = False
        self.gui_blocker.disable()
        self.mode_blocker.disable()
        self.voice_handler('pickup')
        pick_success = self.pickup_outcome.pop(0)
        if len(self.object_list) != 0:
            current_pick_object = self.object_list[0]
        elif len(self.failed_object_list) != 0:
            current_pick_object = self.failed_object_list[0]
        else:
            current_pick_object = 'CUBE'
        self.pick_up_object(pick_obj=current_pick_object, success=pick_success)
        self.voice_handler('approach_ready')
        self.gui_blocker.enable()
        if not self.object_in_hand:
            self.mode_blocker.enable()

        signal.signal(signal.SIGINT, self.signal_handler)
        try:
            while not self.proceed:
                if self.done:
                    self.gui_blocker.enable()
                    return
                elif self.retry:
                    self.retry = False
                    if not self.object_in_hand:
                        self.gui_blocker.disable()
                        self.voice_handler('pickup')
                        pick_success = True # fix mistake
                        self.pick_up_object(pick_obj=current_pick_object, success=pick_success)
                        self.voice_handler('approach_ready')
                        self.mode_blocker.disable()
                    else:
                        self.voice_handler('pickup_error')
                    self.gui_blocker.enable()
                time.sleep(0.1)
        except Exception as e:
            print(f"\nUnexpected error: {e}")
            raise
        
        self.proceed = False
        self.gui_blocker.disable()
        detect_success = self.detect_outcome.pop(0)
        self.voice_handler('detect_wait')
        if not detect_success:
            time.sleep(7)
            self.csv.update_csv(None,None,None,None,'r_detect_error')
            self.voice_handler('detect_error')
            time.sleep(7)
            self.voice_handler()
        self.voice_handler('approach')
        self.approach_human(position="middle", pick_success=pick_success)
        self.object_in_hand = False
        self.gui_blocker.enable()
        self.proceed = False
        self.voice_handler('retreat_ready')
        self.mode_blocker.enable()
        signal.signal(signal.SIGINT, self.signal_handler)
        try:
            while not self.proceed:
                if self.done:
                    self.gui_blocker.enable()
                    return
                elif self.retry:
                    self.gui_blocker.disable()
                    self.voice_handler('retry_error')
                    self.retry = False
                    self.gui_blocker.enable()
                time.sleep(0.1)
        
        except Exception as e:
            print(f"\nUnexpected error: {e}")
            raise
        self.voice_handler() 
        self.gui_blocker.disable()
        self.proceed = False
        self.obj.retreat()
        self.gui_blocker.enable()

        if self.done:
            self.gui_blocker.enable()
            return

    def manual_loop(self):
        self.obj.helper.recovery_client()
        if not self.task_running:
            self.task_running = True
            self.gui_blocker.disable()
            self.obj.go_to_home()
            self.pose = "home"
            self.gui_blocker.enable()
            self.mode_blocker.enable()

        elif self.pickup and self.object!=None and self.speed!=None:
            self.mode_blocker.disable()
            if self.object_in_hand:
                self.voice_handler('pickup_error')
                self.gui_blocker.enable()
            else:
                self.gui_blocker.disable()
                self.voice_handler('pickup')
                pick_success = self.pickup_outcome.pop(0)
                outcome = self.pick_up_object(pick_obj=self.object.upper(), success=pick_success, speed=self.speed)
                if not outcome:
                    self.mode_blocker.enable()
                self.pose = "pickup"
                self.voice_handler() 
                self.gui_blocker.enable()
            self.pickup = False
            self.object = None
            self.speed = None
        
        elif self.approach and self.approach_pos!=None and self.pose!="approach" and self.speed!=None:
            self.gui_blocker.disable()
            self.voice_handler('approach')
            self.approach_human(position=self.approach_pos, pick_success=self.object_in_hand, speed=self.speed)
            self.approach = False
            self.approach_pos = None
            self.speed = None
            self.pose = "approach"
            self.object_in_hand = False
            self.mode_blocker.enable()
            self.voice_handler()
            self.gui_blocker.enable()

        elif self.retreat and self.pose != "home" and self.speed!=None:
            self.gui_blocker.disable()
            self.voice_handler('retreat')
            self.obj.retreat(speed=self.speed)
            self.retreat = False
            self.voice_handler()
            self.gui_blocker.enable()
            self.pose = "home"
            self.speed = None
            if not self.object_in_hand:
                self.mode_blocker.enable()
        
        else:
            self.gui_blocker.enable()

        if self.mode != 'manual':
            self.task_running = False
            self.gui_blocker.enable()
            return
    
    def pick_up_object(self, pick_obj="CUBE", success=True, speed='medium'):
        print('selected speed: '+speed)
        outcome = self.obj.object_prep(pick_obj, success=success, speed=speed)
        if outcome is not None: 
            if pick_obj in self.object_list:
                self.object_list.remove(pick_obj)
            elif pick_obj in self.failed_object_list:
                self.failed_object_list.remove(pick_obj)
            if not outcome:
                self.failed_object_list.append(pick_obj)
            self.object_in_hand = outcome
        else: 
            self.csv.update_csv(None,None,None,None,'r_object_not_found: '+pick_obj)
            self.gui_blocker.enable()
            self.object_in_hand = False
        print('Object list: '+str(self.object_list))
        print('Failed object list: '+str(self.failed_object_list))
        print('Object in hand: '+str(self.object_in_hand))
        return outcome
    
    def approach_human(self, position = "middle", pick_success = True, speed = 'medium'):
        if self.mode == "manual":
            self.mux.select_input("phase_publisher_t")
        else:
            self.mux.select_input("phase_publisher")
        current_pose = self.obj.commander.get_current_pose(end_effector_link = "panda_hand_tcp")
        
        if position == "middle":
            print("set end point to middle")
            rospy.set_param('final_x', middle_end_point[0])
            rospy.set_param('final_y', middle_end_point[1])
            rospy.set_param('final_z', middle_end_point[2])
            rospy.set_param('B_A_ratio', 3.8399)
            rospy.set_param('skewed_factor_single', 0.4)
        elif position == "left":
            print("set end point to left")
            rospy.set_param('final_x', left_end_point[0])
            rospy.set_param('final_y', left_end_point[1])
            rospy.set_param('final_z', left_end_point[2])
            rospy.set_param('B_A_ratio', 3.8399)
            rospy.set_param('skewed_factor_single', 0.4)
        elif position == "right":
            print("set end point to right")
            rospy.set_param('final_x', right_end_point[0])
            rospy.set_param('final_y', right_end_point[1])
            rospy.set_param('final_z', right_end_point[2])
            rospy.set_param('B_A_ratio', 2)
            rospy.set_param('skewed_factor_single', 0.2)

        rospy.set_param('init_x', current_pose.pose.position.x)
        rospy.set_param('init_y', current_pose.pose.position.y)
        rospy.set_param('init_z', current_pose.pose.position.z)
        
        if speed == 'slow':
            rospy.set_param('traj_time', 7.0)
        elif speed == 'medium':
             rospy.set_param('traj_time', 5.0)
        elif speed == 'fast':
            rospy.set_param('traj_time', 3.5)
        
        self.csv.update_csv(None,None,None,None,'r_approach')
        self.obj.handover(int(0), pick_success)
        self.obj.helper.stop_gripper()
        self.object_in_hand = False
    
    def voice_handler(self, msg=''):
        if msg=='':
            self.msg_publisher.send_msg("\n")
        elif msg=='pickup':
            print("Picking up object...")
            if self.transparency:
                self.msg_publisher.send_msg("Picking up object...")
                time.sleep(1)
                playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
        elif msg=='approach':
            print("Approaching...")
            if self.transparency:
                self.msg_publisher.send_msg("Approaching...")
                time.sleep(1)
                playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
        elif msg=='retreat':
            print("Retreating...")
            if self.transparency:
                self.msg_publisher.send_msg("Retreating...")
                time.sleep(1)
                playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
        elif msg=='pickup_ready':
            print("Ready to pick up object")
            self.msg_publisher.send_msg("Ready to pick up object")
            time.sleep(1)
            playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
        elif msg=='approach_ready':
            print("Ready to approach...")
            self.msg_publisher.send_msg("Ready to approach...")
            time.sleep(1)
            playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
        elif msg=='retreat_ready':
            print("Ready to retreat...")
            self.msg_publisher.send_msg("Ready to retreat...")
            time.sleep(1)
            playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
        elif msg=='pickup_error':
            print("ERROR: cannot pick up another object")
            if self.transparency:
                self.msg_publisher.send_msg("Error: cannot pick up another object.")
                time.sleep(1)
                playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
        elif msg=='retry_error':
            print("ERROR: cannot retry previous action")
            if self.transparency:
                self.msg_publisher.send_msg("Error: cannot retry previous action.")
                time.sleep(1)
                playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
        elif msg=='detect_error':
            print("ERROR: did not detect hand")
            if self.transparency:
                self.msg_publisher.send_msg("Error: I cannot see your hand")
                time.sleep(1)
                playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
        elif msg=='detect_wait':
            print("Waiting until hand is visible...")
            if self.transparency:
                self.msg_publisher.send_msg("Waiting until hand is visible...")
                time.sleep(1)
                playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
        elif msg=='done':
            print("Completed picking all objects...")
            if self.transparency:
                self.msg_publisher.send_msg("Completed picking all objects")
                time.sleep(1)
                playsound(f'/home/demo_ws/src/user_studies/audio/sound_files/{msg}.mp3')
    
    def gui_callback(self, data):
        self.last_btn = data.data
        if data.data in ["autonomous","semi_autonomous","manual"]:
            self.mode = data.data
            self.obj.mode = data.data
            print("mode: "+data.data)
        elif data.data == "proceed":
            self.proceed = True
            self.retry = False
        elif data.data == 'retry':
            self.retry = True
            self.proceed = False
        elif data.data == "pickup":
            self.pickup = True
            self.approach = False
            self.retreat = False
            self.approach_pos = None
            self.speed = None
        elif data.data == "approach":
            self.approach = True
            self.pickup = False
            self.retreat = False
            self.speed = None
        elif data.data == "retreat":
            self.retreat = True
            self.pickup = False
            self.approach = False
            self.approach_pos = None
            self.speed = None
        elif data.data in ["Sugar","Dock","Coffee","Tea","Speaker"]:
            self.object = data.data.upper()
            print("object: "+str(self.object))
        elif data.data in ["left","middle","right"]:
            self.approach_pos = data.data
        elif data.data == 'confirm':
            self.reset()
            self.object_list = ['SUGAR','DOCK','SPEAKER','COFFEE','TEA']
            self.failed_object_list = []
        elif data.data == 'simple_back':
            self.gui_blocker.enable()
        elif data.data == 'back':
            if self.mode != 'autonomous':
                self.gui_blocker.enable()
            self.reset()
        elif data.data in ['slow','medium','fast']:
            print('new speed: '+data.data)
            self.speed = data.data
    
    def reset(self):
        self.task_running = False
        self.speed = 'medium'
        self.mode = None
        self.done = True
        self.object_in_hand = False
        self.object = None
        self.pickup = False
        self.approach = False
        self.retreat = False
        self.proceed = False
        self.retry = False
        self.approach_pos = None
        
@app.command()
def main(vision: bool = typer.Option(default = True, help = "Use the Realsense or not"),
        warning: bool = typer.Option(default = True, help = "Warn before moving or not"),
        training: bool = typer.Option(default = False, help = "Training session or not"),
        transparency: bool = typer.Option(default = True, help = "Transparent robot or not")):

    rospy.init_node('trust_study')
    run_obj = HandoverDemo(vision = vision, warning = warning, transparency = transparency)
    study_loop = StudyLoop(run_obj, transparency = transparency, training=training)
    
    while not rospy.is_shutdown():
        study_loop.run_loop()

if __name__ == "__main__":
    app()