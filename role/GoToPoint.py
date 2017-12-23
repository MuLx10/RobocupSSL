from enum import Enum
import behavior
import _GoToPoint
import rospy
from utils.math_functions import *
from utils.config import *

class GoToPoint(behavior.Behavior):
    """docstring for GoToPoint"""
    ##
    ## @brief      Class for state.
    ##
    class State(Enum):
        setup = 1 
        drive = 2

    ##
    ## @brief      Constructs the object.
    ##
    ## @param      self   The object
    ## @param      point  The point
    ##
    def __init__(self,continuous=False):
        # print "gtp"
        #GoToPoint.behavior.Behavior()
        #g = behavior.Behavior()
        #print "gtp2"
        super(GoToPoint,self).__init__()
        #self.state = state


        self.add_state(GoToPoint.State.setup,
            behavior.Behavior.State.running)
        self.add_state(GoToPoint.State.drive,
            behavior.Behavior.State.running)
        

        self.add_transition(behavior.Behavior.State.start,
            GoToPoint.State.setup,lambda: True,'immediately')

        self.add_transition(GoToPoint.State.setup,
            GoToPoint.State.drive,lambda: self.target_present,'setup')

        self.add_transition(GoToPoint.State.drive,
            GoToPoint.State.drive,lambda: not self.at_new_point(),'restart')

        self.add_transition(GoToPoint.State.drive,
            behavior.Behavior.State.completed,lambda:self.at_new_point(),'complete')

    ##
    ## @brief      { function_description }
    ##
    ## @param      self  The object
    ##
    ## @return     { description_of_the_return_value }
    ##
    def add_point(self,point,orient):
        self.target_point = point
        # for i in xrange(10):
        #     print self.target_point
        self.theta = orient
        
    def add_kub(self,kub):
        self.kub = kub
        
    def target_present(self):
        return self.target_point is not None

    ##
    ## @brief      { function_description }
    ##
    ## @return     { description_of_the_return_value }
    ##
    def at_new_point(self):
        #print (dist(self.target_point,self.new_point),210)
        return dist(self.target_point,self.new_point) < DISTANCE_THRESH

        
    def on_enter_setup(self):
        pass
    def execute_setup(self):
        _GoToPoint.init(self.kub,self.target_point,self.theta)
        pass
        
    def on_exit_setup(self):
        pass

    def on_enter_drive(self):
        pass

    def terminate(self):
        super().terminate()
    ##
    ## @brief      { function_description }
    ##
    ## @param      self   The object
    ## @param      kub    The kub
    ## @param      state  The state
    ##
    ## @return     { description_of_the_return_value }
    ##
    def execute_drive(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        t = _GoToPoint.execute(start_time)
        self.new_point = self.kub.get_pos()
        # print self.new_point.x,self.new_point.y
        

    
    def on_exit_drive(self):
        pass



