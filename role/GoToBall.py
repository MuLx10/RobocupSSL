from enum import Enum
import behavior
import _GoToPoint
import rospy
from utils.math_functions import *
from utils.config import *
class GoToBall(behavior.Behavior):
    """docstring for GoToBall"""
    class State(Enum):
        setup = 1 
        course_approach = 2
        fine_approach = 3
    ##
    ## @brief      
    ## Constructs the object.
    ##
    ## @param      self        The object
    ## @param      kub         The kub
    ## @param      theta       The theta
    ## @param      continuous  The continuous
    ##
    def __init__(self,continuous=False):

        super(GoToBall,self).__init__()
        # self.kub = kub

    	self.power = 7.0

        self.add_state(GoToBall.State.setup,
            behavior.Behavior.State.running)

        self.add_state(GoToBall.State.course_approach,
            behavior.Behavior.State.running)
        
        self.add_state(GoToBall.State.fine_approach,
            behavior.Behavior.State.running)

        self.add_transition(behavior.Behavior.State.start,
            GoToBall.State.setup,lambda: True,'immediately')

        self.add_transition(GoToBall.State.setup,
            GoToBall.State.course_approach,lambda: self.target_present(),'setup')

        self.add_transition(GoToBall.State.course_approach,
            GoToBall.State.course_approach,lambda: not self.at_target_point(),'restart')

        self.add_transition(GoToBall.State.course_approach,
            GoToBall.State.fine_approach,lambda:self.at_target_point(),'complete')

        self.add_transition(GoToBall.State.fine_approach,
            GoToBall.State.fine_approach,lambda:not self.at_ball_pos(),'restart')

        self.add_transition(GoToBall.State.fine_approach,
            behavior.Behavior.State.completed,lambda:self.at_ball_pos(),'complete')
    
    def add_kub(self,kub):
        self.kub = kub

    def add_theta(self,theta):
        self.theta = theta
    
    def target_present(self):
        return self.target_point is not None

    def at_target_point(self):
        # print (self.target_point.x,self.target_point.y,self.kub.get_pos(),210)
        # print dist(self.target_point,self.kub.get_pos()) , DISTANCE_THRESH
        return dist(self.target_point,self.kub.get_pos()) < DISTANCE_THRESH

    def at_ball_pos(self):
        # print dist(self.kub.state.ballPos,self.kub.get_pos()) , DISTANCE_THRESH/3
        return dist(self.kub.state.ballPos,self.kub.get_pos()) < DISTANCE_THRESH/3

    def terminate(self):
        super().terminate()
        
    def on_enter_setup(self):
        pass
    def execute_setup(self):
        # self.target_point = self.kub.state.ballPos
        self.target_point = getPointBehindTheBall(self.kub.state.ballPos,self.theta)
        # print self.target_point.x,self.target_point.y
        _GoToPoint.init(self.kub, self.target_point, self.theta)
        pass
        
    def on_exit_setup(self):
        pass

    def on_enter_course_approach(self):
        pass

    def execute_course_approach(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        t = _GoToPoint.execute(start_time)

    def on_exit_course_approach(self):
        pass

    def on_enter_fine_approach(self):
        theta = self.kub.get_pos().theta
        _GoToPoint.init(self.kub, self.kub.state.ballPos, theta)
        pass

    def execute_fine_approach(self):
        start_time = rospy.Time.now()
        start_time = 1.0*start_time.secs + 1.0*start_time.nsecs/pow(10,9)   
        t = _GoToPoint.execute(start_time)

    def disable_kick(self):
    	self.power = 0.0

    def on_exit_fine_approach(self):
    	self.kub.kick(self.power)
    	self.kub.execute()
        pass





