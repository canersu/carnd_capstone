import rospy
from yaw_controller import YawController 
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    #def __init__(self, *args, **kwargs):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        kp=0.3
        ki=0.001
        kd=0.8
        mn=0
        mx=0.2
        self.throttle_controller= PID(kp, ki, kd, mn, mx)
        
        ## Initialize yaw controller::
        self.yaw_controller=YawController(wheel_base, steer_ratio, 0.1, max_lat_accel, max_steer_angle)
        
        ## Initialize low pass filter::
        tau=0.5 
        ts=0.2
        self.lpf=LowPassFilter(tau, ts)
        
        ## Initialize parameters::
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.last_time=rospy.get_time()

    #def control(self, *args, **kwargs):
    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0,0.0,0.0
        
        # Remove high freq noise signals
        curr_vel=self.lpf.filt(curr_vel)
        self.last_vel=curr_vel
        
        # Compute Yaw Controller
        steering =self.yaw_controller.get_steering(linear_vel, angular_vel, curr_vel)

        # Step function
        vel_err=linear_vel-curr_vel
        sample_time=rospy.get_time()- self.last_time
        
        throttle=self.throttle_controller.step(vel_err, sample_time)
        brake=0

        self.last_time=rospy.get_time()
        self.last_vel=curr_vel
        
        # Check braking conditions
        if linear_vel==0 and curr_vel<0.1:
            throttle=0
            brake = 400
            
        if throttle<0.1 and vel_err<0:
            throttle=0
            decel=max(vel_err, self.decel_limit)
            brake= abs(decel)*self.vehicle_mass*self.wheel_radius
        
        return throttle, brake, steering
