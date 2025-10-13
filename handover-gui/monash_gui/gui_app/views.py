from django.shortcuts import render, redirect
from django.http import HttpResponseRedirect
from django.http import HttpResponse
from .csv_functions import CSV
import time
from threading import Thread
import rospy
from std_msgs.msg import String

detect_outcome = [1,1,1,0,1,0,1,1,0,1,1,1,0,1,1] # 25% error
pickup_outcome = [1,1,0,1,0,1,1,0,1,1,1,0,1,1,1] # 25% error

class GuiPublisher():
    def __init__(self):
        rospy.init_node('gui_publisher', anonymous=True)
        self.gui_publisher = rospy.Publisher('gui_btn', String, queue_size=10)
    
    def send_msg(self, msg: str):
        self.gui_publisher.publish(msg)

class ROSThread(Thread):
    def __init__(self):
        super().__init__()
        pass
        
    def run(self):
        pass

class AutoThread(Thread):
    def __init__(self, condition = "transparent"):
        super().__init__()
        self.running = True
        self.counter = 0
        self.objects = ["object1","object2","object3"]
        self.condition = condition
        
    def run(self):
        r = rospy.Rate(60) 
    """
        while self.running and len(self.objects) != 0: 
            print("Objects in workspace: ",self.objects)
            # PICKUP
            if self.condition == "transparent": 
                print("Picking up object.")
            global pickup_outcome
            success = pickup_outcome.pop(0)
            if success:
                print("Robot picks up object correctly...")
            else:
                print("Robot picks up object incorrectly...")

            # APPROACH
            global detect_outcome
            success = detect_outcome.pop(0) 
            if not success: 
                while self.counter < 3: 
                    print("Waiting until hand is visible...") # say out loud if transparent?
                    time.sleep(1)
                    self.counter += 1
                self.counter = 0
                if self.condition == "transparent":
                    print("I cannot see your hand.") # also if counter > N
            while self.counter < 3: 
                time.sleep(1)
                print("Waiting until hand is visible...") # say out loud if transparent?
                self.counter += 1
            self.counter = 0
            if self.condition == "transparent":
                print("I can see your hand. Approaching.")
            print("Robot approaches...")

            # RETREAT
            while self.counter < 3: 
                print("Waiting until Force > T...")
                time.sleep(1)
                self.counter += 1
            self.counter = 0
            if self.condition == "transparent":
                print("retreating")
            print("Robot retreats")
            
            #self.objects.pop() # del
"""
    def stop(self):
        self.running = False
    

class SemiAutoThread(Thread):
    def __init__(self, condition = "transparent"):
        super().__init__()
        self.running = True
        self.counter = 0
        self.objects = ["object1","object2","object3"]
        self.condition = condition
        self.proceed = False
        self.retry_active = False
    
    def run(self):
        pass
    """
        while self.running and len(self.objects) != 0: 
            self.retry_active = False
            print("Objects in workspace: ",self.objects)
            
            # PICKUP
            print("Ready to pick up object")
            while not self.proceed:
                time.sleep(1)
            self.proceed = False
            if self.condition == "transparent": 
                print("Picking up object.")
            global pickup_outcome
            success = pickup_outcome.pop(0)
            if success:
                print("Robot picks up object correctly...")
            else:
                print("Robot picks up object incorrectly...")
                self.retry_active = True
            
            # APPROACH
            global detect_outcome
            success = detect_outcome.pop(0) 
            # First check if hardcoded error
            if not success: 
                while self.counter < 5: 
                    print("Waiting until hand is visible...") # say out loud if transparent?
                    time.sleep(1)
                    self.counter += 1
                self.counter = 0
                if self.condition == "transparent":
                    print("I cannot see your hand.") # also if counter > N
            # Then wait until hand is visible
            while self.counter < 5: 
                print("Waiting until hand is visible...")
                time.sleep(1)
                self.counter += 1
            self.counter = 0
            if self.condition == "transparent":
                print("I can see your hand. Ready to approach.")
                self.proceed = False
            else:
                print("Ready to approach.")
                self.proceed = False
            # Wait for button click
            while not self.proceed:
                time.sleep(1)
            if self.condition == "transparent":
                print("Approaching.")
            print("Robot approaches...")
            
            # RETREAT
            while self.counter < 5: 
                print("Waiting until Force > T...")
                time.sleep(1)
                self.counter += 1
            self.counter = 0
            self.proceed = False
            print("Ready to retreat.")
            while not self.proceed:
                time.sleep(1)
            self.proceed = False
            if self.condition == "transparent":
                print("Retreating.")
            print("Robot retreats...")

            self.objects.pop() # del

    def retry_pickup(self):
        if self.retry_active:
            self.retry_active = False
            if self.condition == "transparent":
                print("Picking up object.")
            print("Robot picks up object correctly...")

            # Then wait until hand is visible
            while self.running and self.counter < 5: 
                print("Waiting until hand is visible...")
                time.sleep(1)
                self.counter += 1
            self.counter = 0
            if self.condition == "transparent":
                print("I can see your hand. Ready to approach.")
                self.proceed = False
            else:
                print("Ready to approach.")
                self.proceed = False
        else:
            print("This action is not possible right now")
     """   
    def stop(self):
        self.running = False
    

class ManualThread(Thread):
    def __init__(self, condition = "transparent"):
        super().__init__()
        self.running = True
        self.counter = 0
        self.objects = ["object1","object2","object3"]
        self.condition = condition
        self.pickup = False
        self.approach = False
        self.retreat = False
        self.obj = "None"
        self.object_in_hand = False

    def run(self):
        pass

    def stop(self):
        self.running = False
"""
        while self.running and len(self.objects) != 0: 
            self.retry_active = False
            #print("Objects in workspace: ",self.objects)
            
            # PICKUP   
            if self.pickup and self.object!="None":
                if self.object_in_hand:
                    print("ERROR: cannot pick up object if still carrying one")
                else:
                    if self.condition == "transparent": 
                        print("Picking up object: "+self.object)
                    global pickup_outcome
                    success = pickup_outcome.pop(0)
                    if success:
                        print("Robot picks up object correctly...")
                        self.object_in_hand = True
                    else:
                        print("Robot picks up object incorrectly...")
                self.pickup = False
                self.object = "None"

            # APPROACH
            elif self.approach:
                if self.condition == "transparent":
                    print("Approaching.")
                print("Robot approaches...")
                self.approach = False
            
            # RETREAT
            elif self.retreat:
                if self.condition == "transparent":
                    print("Retreating.")
                print("Robot retreats...")
                self.retreat = False

                #self.objects.pop() # del

                # TODO: CHECK FOR OBJECT IN HAND (F > T)
                self.object_in_hand = False
"""
        
class Views():
    def __init__(self):
        self.trials = 5
        self.trial_no = 1
        self.condition = "transparent" #"opaque"
        self.participant_id = None
        self.mode = None
        self.csv = CSV()
        self.auto_thread = None
        self.semi_auto_thread = None
        self.manual_thread = None
        self.btn_publisher = GuiPublisher()
        self.msg_listener = rospy.Subscriber("/robot_msg", String, self.msg_callback)
        self.msg = " "
        r = rospy.Rate(60)

    def msg_callback(self, data):
        self.msg = data.data
        print("I heard: "+data.data)
        print(self.msg)
        #if self.mode == 'autonomous':
        #    print('hi')
        #    return redirect('autonomous',msg=self.msg)

    def health_check(self, request):
        return HttpResponse("OK", status=200)

    def begin(self,request):
        self.trial_no = 1
        if request.method == 'POST':
            self.participant_id = request.POST.get('participant_id')
            self.update_csv("begin")
            return redirect('trial_screen', trial=self.trial_no,no_trials=self.trials)
        return render(request, 'gui_app/begin.html')

    def mode_selection(self,request):
        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.mode = decision
            self.update_csv("choose mode")
            if decision == "autonomous":
                self.btn_publisher.send_msg("autonomous")
                return redirect('autonomous',msg=self.msg)
            elif decision == "semi_autonomous":
                self.btn_publisher.send_msg("semi_autonomous")
                return redirect("semi_autonomous",msg=self.msg)
            elif decision == "manual":
                self.btn_publisher.send_msg("manual")
                return redirect("manual",obj="None",msg=self.msg)
        return render(request, 'gui_app/mode_selection.html')

    def autonomous(self,request, msg):
        #if request.method != 'POST':
        #    self.auto_thread = AutoThread(self.condition)
        #    self.auto_thread.start()
        if request.method == 'POST':
            if self.auto_thread:
                self.auto_thread.stop()
            decision = request.POST.get('decision')
            self.btn_publisher.send_msg(decision)
            if decision == "complete":
                self.update_csv("complete pack")
                return redirect("confirm_complete", mode="autonomous")
            elif decision == "back":
                self.update_csv("go back")
                return redirect("mode_selection")
        return render(request, 'gui_app/autonomous.html',{"msg":self.msg})

    def semi_autonomous(self,request, msg):
        #if request.method != 'POST':
        #    self.semi_auto_thread = SemiAutoThread(self.condition)
        #    self.semi_auto_thread.start()

        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.btn_publisher.send_msg(decision)
            self.update_csv(decision)
            if decision == "retry":
                #if self.semi_auto_thread:
                #    self.semi_auto_thread.retry_pickup()
                return render(request, 'gui_app/semi_autonomous.html')
            elif decision == "proceed":
                if self.semi_auto_thread:
                    self.semi_auto_thread.proceed = True
                return render(request, 'gui_app/semi_autonomous.html')
            elif decision == "complete":
                if self.semi_auto_thread:
                    self.semi_auto_thread.stop()
                return redirect("confirm_complete", mode="semi_autonomous")
            elif decision == "back":
                if self.semi_auto_thread:
                    self.semi_auto_thread.stop()
                return redirect("mode_selection")
        return render(request, 'gui_app/semi_autonomous.html',{"msg":self.msg})

    def manual(self, request, obj, msg):
        #if request.method != 'POST':
            #if not self.manual_thread:
            #    self.manual_thread = ManualThread(self.condition)
            #    self.manual_thread.start()
            #self.manual_thread.obj = obj
            #self.manual_thread.pickup = True

        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.btn_publisher.send_msg(decision)
            self.update_csv(decision)
            if decision == "pickup":
                return redirect("choose_object")
            elif decision == "approach":
                #self.manual_thread.approach = True
                return render(request, 'gui_app/choose_approach_position.html')
            elif decision == "retreat":
                #self.manual_thread.retreat = True
                return render(request, 'gui_app/manual.html')
            elif decision == "complete":
                #if self.manual_thread:
                #    self.manual_thread.stop()
                return redirect("confirm_complete", mode="manual")
            elif decision == "back":
                if self.manual_thread:
                    self.manual_thread.stop()
                return redirect("mode_selection")
        return render(request, 'gui_app/manual.html',{"obj":obj,"msg":msg})

    def choose_object(self, request):
        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.btn_publisher.send_msg(decision)
            self.update_csv(decision)
            if decision == "back":
                return redirect("manual",obj="None",msg=self.msg)
            elif decision in ["1","2","3","4","5"]:
                return redirect("manual", obj=decision, msg=self.msg)
        return render(request, 'gui_app/choose_object.html')
        
    def choose_approach_position(self, request):
        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.btn_publisher.send_msg(decision)
            self.update_csv(decision)
            if decision == "back":
                return redirect("manual",obj="None",msg=self.msg)
            elif decision in ["left","middle","right"]:
                return redirect("manual", obj="None",msg=self.msg)
        return render(request, 'gui_app/choose_approach_position.html')
        
    def confirm_complete(self, request, mode):
        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.btn_publisher.send_msg(decision)
            self.update_csv(decision)
            if decision == "confirm":
                self.trial_no += 1
                if self.trial_no <= self.trials:
                    return redirect("trial_screen", trial=self.trial_no, no_trials=self.trials)
                else:
                    return redirect("end_screen")
            elif decision == "back":
                return redirect(mode)
        return render(request, 'gui_app/confirm_complete.html',{"mode":mode})

    def trial_screen(self, request, trial, no_trials):
        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.update_csv("start trial")
            if decision == "ok":
                return redirect("mode_selection")
        return render(request, 'gui_app/trial_screen.html',{"trial":trial,"no_trials":no_trials})

    def end_screen(self, request):
        global trial
        trial = 1
        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.btn_publisher.send_msg(decision)
            self.update_csv("end task")
            if decision == "ok":
                return redirect("begin")
        return render(request, 'gui_app/end_screen.html')

    def debug_navigation(self, request):
        print(f"Current URL: {request.path}")
        print(f"Request method: {request.method}")
        print(f"POST data: {request.POST}")
        print(f"Referer: {request.META.get('HTTP_REFERER')}")
        return redirect("mode_selection")

    def your_view(self, request):
        print(f"Request IP: {request.META.get('REMOTE_ADDR')}")
        print(f"Request method: {request.method}")
        print(f"Request path: {request.path}")

    def update_csv(self, action: str):
        self.csv.update_csv(participant_id=self.participant_id,condition=self.condition,trial_no=self.trial_no,
                            mode=self.mode,action=action)

    