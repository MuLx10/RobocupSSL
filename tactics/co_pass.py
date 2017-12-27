import rospy
import composite_behavior
import behavior
import enum

from role import KickToPoint, BallReceiver



class CoordinatedPass(composite_behavior.CompositeBehavior):

    class State(enum.Enum):
        preparing = 1  # the kicker is aiming and the receiver is getting ready
        kicking = 2  # waiting for the kicker to kick
        receiving = 3  # the kicker has kicked and the receiver is trying to get the ball
        timeout = 4

    ## Init method for CoordinatedPass
    # @param ballReceiver an instance of a class that will handle the receiving robot. See pass_receive and angle_receive for examples.
    # Using this, you can change what the receiving robot does (rather than just receiving the ball, it can pass or shoot it).
    # Subclasses of pass_receive are preferred, but check the usage of this variable to be sure.
    # @param receive_point The point that will be kicked too. (Target point)
    # @param kickToPoint A tuple of this form (kicking_class instance, ready_lambda). If none, it will use (pivot_kick lambda x: x == pivot_kick.State.aimed).
    # @param receiver_required Whether the receiver subbehavior should be required or not
    # @param kicker_required Whether the kicker subbehavior should be required or not
    # The lambda equation is called (passed with the state of your class instance) to see if your class is ready. Simple implementations will just compare it to your ready state.
    def __init__(self, kicker_kub=None, receiver_kub=None, receive_point=None, prekick_timeout=None):
        super(CoordinatedPass, self).__init__(continuous=False)

        self.receive_point = receive_point
        self.ballReceiver = BallReceiver(self.kicker_kub)
        self.kickToPoint = (KickToPoint(self.receiver_kub),lambda x: x == skills.pivot_kick.kickToPoint.State.aimed)
        self.prekick_timeout = prekick_timeout

        self.add_state(CoordinatedPass.State.preparing,
                       behavior.Behavior.State.running)
        self.add_state(CoordinatedPass.State.kicking,
                       behavior.Behavior.State.running)
        self.add_state(CoordinatedPass.State.receiving,
                       behavior.Behavior.State.running)
        self.add_state(CoordinatedPass.State.timeout,
                       behavior.Behavior.State.failed)

        self.add_transition(behavior.Behavior.State.start,
                            CoordinatedPass.State.preparing, lambda: True,
                            'immediately')

        self.add_transition(
            CoordinatedPass.State.preparing, CoordinatedPass.State.kicking,
            lambda: (kickToPoint[1](self.subbehavior_with_name('kicker').state) and self.subbehavior_with_name('receiver').state == self.ballReceiver.State.aligned),
            'kicker and receiver ready')

        self.add_transition(
            CoordinatedPass.State.preparing, CoordinatedPass.State.timeout,
            self.prekick_timeout_exceeded, 'Timed out on prepare')

        self.add_transition(CoordinatedPass.State.kicking,
                            CoordinatedPass.State.timeout,
                            self.prekick_timeout_exceeded, 'Timed out on kick')

        self.add_transition(
            CoordinatedPass.State.kicking, CoordinatedPass.State.receiving,
            lambda: self.subbehavior_with_name('kicker').state == behavior.Behavior.State.completed,
            'kicker kicked')

        self.add_transition(
            CoordinatedPass.State.receiving, behavior.Behavior.State.completed,
            lambda: self.subbehavior_with_name('receiver').state == behavior.Behavior.State.completed,
            'pass received!')

        self.add_transition(
            CoordinatedPass.State.receiving, behavior.Behavior.State.failed,
            lambda: self.subbehavior_with_name('receiver').state == behavior.Behavior.State.failed,
            'pass failed :(')

    
    def add_kicker_kub(self, kub):
        self.kicker_kub = kub

    def add_receiver_kub(self, kub):
        self.receiver_kub = kub

    def getCurrTime(self):
        start_time = rospy.Time.now()
        return start_time = 1.0 * start_time.secs + 1.0 * start_time.nsecs / pow(10, 9)


    # set the location where the receiving bot should camp out and wait for the ball
    # Default: None
    @property
    def receive_point(self):
        return self._receive_point

    @receive_point.setter
    def receive_point(self, value):
        self._receive_point = value

        # set receive_point for kicker and receiver (if present)
        if self.has_subbehavior_with_name('kicker'):
            self.subbehavior_with_name('kicker').target = self.receive_point
        if self.has_subbehavior_with_name('receiver'):
            self.subbehavior_with_name(
                'receiver').receive_point = self.receive_point

    def on_enter_running(self):
        receiver = self.ballReceiver
        receiver.receive_point = self.receive_preceive_pointoint
        self.add_subbehavior(receiver,
                             'receiver',
                             required=True)

    def on_exit_running(self):
        self.remove_subbehavior('receiver')

    def on_enter_kicking(self):
        self.subbehavior_with_name('kicker').enable_kick = True

    def on_enter_preparing(self):
        kicker = self.kickToPoint[0]
        kicker.target = self.receive_point
        kickpower = 6
        kicker.kick_power = kickpower
        kicker.enable_kick = False  # we'll re-enable kick once both bots are ready

        # we use tighter error thresholds because passing is hard
        kicker.aim_params['error_threshold'] = 0.2
        kicker.aim_params['max_steady_ang_vel'] = 3.0
        kicker.aim_params['min_steady_duration'] = 0.15
        kicker.aim_params['desperate_timeout'] = 3.0
        self.add_subbehavior(kicker, 'kicker', required=True)

        # receive point renegotiation
        self._last_unsteady_time = None
        self._has_renegotiated_receive_point = False

        self._preparing_start = self.getCurrTime()

    def execute_running(self):
        # The shot obstacle doesn't apply to the receiver
        if self.has_subbehavior_with_name('kicker'):
            kicker = self.subbehavior_with_name('kicker')
            receiver = self.subbehavior_with_name('receiver')
            kicker.shot_obstacle_ignoring_robots = [receiver.robot]

    def execute_preparing(self):
        kicker = self.subbehavior_with_name('kicker')

        # receive point renegotiation
        # if the kicker sits there aiming close to target and gets stuck,
        # we set the receive point to the point the kicker is currently aiming at
        if kicker.current_shot_point(
        ) != None and not self._has_renegotiated_receive_point:
            if (not kicker.is_steady() and self.kickToPoint[1](kicker.state)):
                self._last_unsteady_time = self.getCurrTime()

            if (self._last_unsteady_time != None and
                    self.getCurrTime() - self._last_unsteady_time > 0.75 and
                    kicker.current_shot_point().dist_to(self.receive_point) <
                    0.1):
                # renegotiate receive_point
                print("Pass renegotiated RCV PT")
                self.receive_point = kicker.current_shot_point()
                self._has_renegotiated_receive_point = True

    def prekick_timeout_exceeded(self):
        if self._preparing_start == None or self.prekick_timeout == None or self.prekick_timeout <= 0:
            return False
        if self.getCurrTime() - self._preparing_start > self.prekick_timeout:
            return True
        return False

    def time_remaining(self):
        if self._preparing_start == None or self.prekick_timeout == None or self.prekick_timeout <= 0:
            return 0
        return self.prekick_timeout - (self.getCurrTime() - self._preparing_start)

    def on_enter_receiving(self):
        # once the ball's been kicked, the kicker can go relax or do another job
        self.subbehavior_with_name('receiver').ball_kicked = True
        self.remove_subbehavior('kicker')
