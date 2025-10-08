import csv
import time

class CSV():
    def __init__(self):
        self.filename = "experiment_data.csv"
        fields = ['ID', 'Condition', 'Trial', 'Mode', 'Action', 'Time']
        with open(self.filename, 'w') as csvfile: # remember to change to 'a'
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(fields)

    def update_csv(self, participant_id, condition, trial_no, mode, action):
        timestamp = time.time()
        with open(self.filename, 'a') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow([participant_id, condition, trial_no, mode, action, timestamp])