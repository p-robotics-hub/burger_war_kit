import requests
import json

class JudgeReceiver:
    def urlreq(self):
        resp = requests.get("http://localhost:5000/warState")
        return resp.text

    def getStatus(self):
        state_json = self.urlreq()
        self.judge_str = json.loads(state_json)

        time_string=self.judge_str['time']
        self.time=float(time_string)
        
        self.state=self.judge_str['state']

        self.team_blue=self.judge_str['players']['b']
        self.team_red=self.judge_str['players']['r']

        self.point_blue=self.judge_str['scores']['b']
        self.point_red=self.judge_str['scores']['r']

        self.team_blue_ready=self.judge_str['ready']['b']
        self.team_red_ready=self.judge_str['ready']['r']

        self.targets=self.judge_str['targets']