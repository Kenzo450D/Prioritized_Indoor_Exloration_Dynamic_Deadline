"""
====
Idea
====
The exploration clock are t_ao timers which are set in prior. 
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
    def __init__(self, t_a, t_r_0):
        # t_a is the exploration time before whistle
        # tb is the exploration time after whistle
        self.t_a = t_a
        self.t_r_0 = t_r_0 # set up initial deadline
        self.t_r = t_r_0 # set up current deadline as t_r
        if t_a== 0:
            self.time_limit_active = True
        else:
            self.time_limit_active = False
    
    def get_initial_deadline(self):
        return self.t_r_0

    def get_total_exploration_time(self):
        return self.t_a + self.t_r_0

    def get_time_limit(self):
        if self.time_limit_active:
            return self.t_r
        else:
            return None
    
    def update_time_limit(self):
        self.t_a -= 1
        if self.t_a > 0: # time before deadline 
            return self.get_time_limit()
        elif self.t_a == 0: # time before deadline is consumed
            self.time_limit_active = True
            return self.get_time_limit()
        else:
            self.time_limit_active = True
            self.t_r -= 1
            if self.t_r < 0:
                print ("Time Limit Exceeded for t_r")
                raise ValueError
            return self.get_time_limit()
    
        
        
            
