"""
====
Idea
====
The exploration clock are two timers which are set in prior. 
Timer 1 of 2:
    Exploration without time limit clock. This clock is unavailable  to the 
    exploration agent. And the agent only gets to know of it when the 
    exploration is over.
Timer 2 of 2:
    This is the time which is the agent is aware of. The agent observes this
    timer and acts accordingly to make sure that the agent can return to base
    within the limited amount of time it has at its disposal.

=======================
Implementation Details:
=======================
Class 'ExplorationClock' handles 

"""
import sys

class ExplorationClock:
    def __init__(self, tw, tb):
        # tw is the exploration time before whistle
        # tb is the exploration time after whistle
        self.tw = tw
        self.tb = tb
        if tw== 0:
            self.time_limit_active = True
        else:
            self.time_limit_active = False
    
    def get_time_limit(self):
        if self.time_limit_active:
            return self.tb
        else:
            return None
    
    def update_time_limit(self):
        self.tw -= 1
        if self.tw > 0:
            #print ("function: update_time_limit: :: tw: {} \ttb: {}".format(self.tw, self.tb))
            #print ("Time limit active: ", self.time_limit_active)
            return self.get_time_limit()
        elif self.tw == 0:
            #print ("Setting time limit active, no change in tb")
            self.time_limit_active = True
            #print ("function: update_time_limit: :: tw: {} \ttb: {}".format(self.tw, self.tb))
            #print ("Time limit active: ", self.time_limit_active)
            return self.get_time_limit()
        else:
            self.time_limit_active = True
            #print ("Time limit already active, changing tb")
            self.tb -= 1
            #print ("function: update_time_limit: :: tw: {} \ttb: {}".format(self.tw, self.tb))
            #print ("Time limit active: ", self.time_limit_active)
            if self.tb < 0:
                print ("Time Limit Exceeded for Tb")
                #raise ValueError
            return self.get_time_limit()
    
        
        
            
