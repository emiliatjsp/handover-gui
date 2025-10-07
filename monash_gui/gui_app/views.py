from django.shortcuts import render, redirect
from django.http import HttpResponseRedirect
from django.http import HttpResponse


TRIALS = 10
trial = 1

def health_check(request):
    return HttpResponse("OK", status=200)

def begin(request):
    global trial
    trial = 1
    if request.method == 'POST':
        participant_id = request.POST.get('participant_id')
        return redirect('trial_screen', trial=trial,no_trials=TRIALS)
    return render(request, 'gui_app/begin.html')

def mode_selection(request):
    if request.method == 'POST':
        decision = request.POST.get('decision')
        if decision == "autonomous":
            return redirect('autonomous')
        elif decision == "semi_autonomous":
            return redirect("semi_autonomous")
        elif decision == "manual":
            return redirect("manual")
    return render(request, 'gui_app/mode_selection.html')

def autonomous(request):
    if request.method == 'POST':
        decision = request.POST.get('decision')
        if decision == "complete":
            return redirect("confirm_complete", mode="autonomous")
        elif decision == "back":
            return redirect("mode_selection")
    return render(request, 'gui_app/autonomous.html')

def semi_autonomous(request):
    if request.method == 'POST':
        decision = request.POST.get('decision')
        if decision == "retry":
            return render(request, 'gui_app/semi_autonomous.html')
        elif decision == "proceed":
            return render(request, 'gui_app/semi_autonomous.html')
        elif decision == "complete":
            return redirect("confirm_complete", mode="semi_autonomous")
        elif decision == "back":
            return redirect("mode_selection")
    return render(request, 'gui_app/semi_autonomous.html')

def manual(request):
    if request.method == 'POST':
        decision = request.POST.get('decision')
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

def choose_object(request):
    if request.method == 'POST':
        decision = request.POST.get('decision')
        if decision == "back":
            return redirect("manual")
        elif decision in ["1","2","3","4","5"]:
            return redirect("manual")
    return render(request, 'gui_app/choose_object.html')

def confirm_complete(request, mode):
    if request.method == 'POST':
        decision = request.POST.get('decision')
        if decision == "confirm":
            global trial
            trial += 1
            if trial <= TRIALS:
                return redirect("trial_screen", trial=trial, no_trials=TRIALS)
            else:
                return redirect("end_screen")
        elif decision == "back":
            return redirect(mode)
    return render(request, 'gui_app/confirm_complete.html',{"mode":mode})

def trial_screen(request, trial, no_trials):
    if request.method == 'POST':
        decision = request.POST.get('decision')
        if decision == "ok":
            return redirect("mode_selection")
    return render(request, 'gui_app/trial_screen.html',{"trial":trial,"no_trials":no_trials})

def end_screen(request):
    global trial
    trial = 1
    if request.method == 'POST':
        decision = request.POST.get('decision')
        if decision == "ok":
            return redirect("begin")
    return render(request, 'gui_app/end_screen.html')

def debug_navigation(request):
    print(f"Current URL: {request.path}")
    print(f"Request method: {request.method}")
    print(f"POST data: {request.POST}")
    print(f"Referer: {request.META.get('HTTP_REFERER')}")
    return redirect("mode_selection")

def your_view(request):
    print(f"Request IP: {request.META.get('REMOTE_ADDR')}")
    print(f"Request method: {request.method}")
    print(f"Request path: {request.path}")