from django.shortcuts import render, redirect
from django.http import HttpResponseRedirect
from django.http import HttpResponse
from .csv_functions import CSV

class Views():
    def __init__(self):
        self.trials = 5
        self.trial_no = 1
        self.condition = "transparent" #"opaque"
        self.error = [0,0,0,1,0,1,0,0,1,0]
        self.participant_id = None
        self.mode = None
        self.csv = CSV()

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
                return redirect('autonomous')
            elif decision == "semi_autonomous":
                return redirect("semi_autonomous")
            elif decision == "manual":
                return redirect("manual")
        return render(request, 'gui_app/mode_selection.html')

    def autonomous(self,request):
        if request.method == 'POST':
            decision = request.POST.get('decision')
            # TODO: 
                # while objects in workspace:
                    # if transparent:
                        # "picking up object"
                    # pick up object
                    # if failure:
                        # while hand not visible:
                            # sleep
                        # if mode == transparent
                            # "I cannot see your hand"
                    # while hand not visible:
                        # sleep
                    # if mode transparent:
                        # "I can see your hand. Approaching."
                    # approach
            if decision == "complete":
                self.update_csv("complete pack")
                return redirect("confirm_complete", mode="autonomous")
            elif decision == "back":
                self.update_csv("go back")
                return redirect("mode_selection")
        return render(request, 'gui_app/autonomous.html')

    def semi_autonomous(self,request):
        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.update_csv(decision)
            if decision == "retry":
                return render(request, 'gui_app/semi_autonomous.html')
            elif decision == "proceed":
                return render(request, 'gui_app/semi_autonomous.html')
            elif decision == "complete":
                return redirect("confirm_complete", mode="semi_autonomous")
            elif decision == "back":
                return redirect("mode_selection")
        return render(request, 'gui_app/semi_autonomous.html')

    def manual(self,request):
        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.update_csv(decision)
            if decision == "pickup":
                return redirect("choose_object")
            elif decision == "approach_left":
                return render(request, 'gui_app/manual.html')
            elif decision == "approach_right":
                return render(request, 'gui_app/manual.html')
            elif decision == "complete":
                return redirect("confirm_complete", mode="manual")
            elif decision == "back":
                return redirect("mode_selection")
        return render(request, 'gui_app/manual.html')

    def choose_object(self,request):
        if request.method == 'POST':
            decision = request.POST.get('decision')
            self.update_csv(decision)
            if decision == "back":
                return redirect("manual")
            elif decision in ["1","2","3","4","5"]:
                return redirect("manual")
        return render(request, 'gui_app/choose_object.html')

    def confirm_complete(self, request, mode):
        if request.method == 'POST':
            decision = request.POST.get('decision')
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