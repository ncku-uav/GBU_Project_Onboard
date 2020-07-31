#!/usr/bin/env python
# license removed for brevity
import message_filters
import rospy, time
from std_msgs.msg import String,Float64,Bool
from geometry_msgs.msg import TwistStamped
import numpy as np
from numpy import cos,sin,tan
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import ActuatorControl, State, GlobalPositionTarget
from wgs_conversions.srv import WgsConversion


class gbu_drop():
    def __init__(self):
        self.node_name = rospy.get_name()
        # Publicaiton
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
        self.cmd_pub = rospy.Publisher('/mavros/actuator_control', ActuatorControl, queue_size=1)      
        self.bomb_status_pub = rospy.Publisher('bomb_status',Bool, queue_size=1)
        # Subscriptions
        self.gps_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix,self.gps_cb) 
        self.alt_sub = rospy.Subscriber('/mavros/global_position/rel_alt', Float64,self.alt_cb) 
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped,self.vel_cb)
        self.gpsvel_sub = rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped,self.gpsvel_cb)
        self.state_sub = rospy.Subscriber('/mavros/state', State, self.state_cb)

        # Setup parameters
        self.setParams()

        self.pos =[0,0,0]   #x, y ,v  for calculate the distance
        self.vel =[0,0,0]
        self.gps_vel = [0,0,0]
        self.dis_2_target = 0
        self.gps = NavSatFix()
        self.mode = "AUTO.LOITER"
        self.alt = 0.0
        self.wpGps = GlobalPositionTarget()
        self.bomb_release = False
        self.servo_delay = 0.125  # 1/8 secs

    def state_cb(self,state):        
        self.mode = state.mode

    def gps_cb(self,gps):     
        self.gps = gps


    def alt_cb(self,alt):     
        self.alt =  alt.data

    def vel_cb(self,vel): 
        self.vel = [vel.twist.linear.x ,vel.twist.linear.y ,vel.twist.linear.z ]

    def gpsvel_cb(self,vel): 
        self.gps_vel = [vel.twist.linear.x ,vel.twist.linear.y ,vel.twist.linear.z ]
        

    def calDropDistance(self,h,v_):
     
        displacement_total = 0
        v = np.array([v_[0],v_[2],v_[1]])
        
        while h >= 0.0:     
            h += v[1]*self.dt
            V=(np.dot(v,v))**0.5
            drag=0.5*self.air_density*self.area*(V**2)
            v = v +np.array([0,-self.g*self.dt,0] )- self.dt*drag/V/self.mass*v
            displacement_total +=np.dot(v,[1.0, 0.0 , 0.0])*self.dt
            
            #print(h,self.alt)
        displacement_total += self.vel[0] *self.servo_delay

        return displacement_total

     

    def setParams(self):
        g=rospy.get_param("~g",9.8 )                     
        air_drag_coe = rospy.get_param("~air_drag_coe",0.3 )
        mass=rospy.get_param("~mass",0.536)
        area=rospy.get_param("~rea",0.004)
        air_density=rospy.get_param("~air_density",1.225)

        dt = rospy.get_param("~dt",0.001)
        target = rospy.get_param("~target",[0.0,0.0,0.0])
        D = rospy.get_param("~D",10) #(m)
        C = rospy.get_param("~C",10) #(m)
        B = rospy.get_param("~B",1.2) #(m)
        self.target = self.setupParameter("target",target) #target position in GPS
        self.g = self.setupParameter("g",g) # gravity
        self.air_drag_coe = self.setupParameter("air_drag_coe",air_drag_coe) #drag coefficient of bomb
        self.mass = self.setupParameter("mass",mass) #mass of bomb
        self.area = self.setupParameter("area",area) #reference area of bomb
        self.air_density = self.setupParameter("air_density",air_density) #air density
        self.dt = self.setupParameter("dt",dt) #time step in calDropDistance
        self.D = self.setupParameter("D",D ) #direction accuracy
        self.C = self.setupParameter("C",C ) #heading calibration waypoint distance
        self.B = self.setupParameter("B",B )
    
    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value



def rot_pos(x,y,phi):

    return np.array((x*np.cos(phi)+y*np.sin(phi), -x*np.sin(phi)+y*np.cos(phi)))

if __name__ == '__main__':
    
    rospy.init_node('gbu_release_node', anonymous=False)
    rate = rospy.Rate(50)# 10hz
    n = gbu_drop()
    #j =1
    #wp = []
    #wp_idx = 0
    ref_lat = n.target[0]
    ref_long = n.target[1]
    ref_alt = n.target[2]  # ETHZ case

    bomb_status = Bool()
    bomb_status.data = n.bomb_release

    # Send a servo lock signal
    cmd = ActuatorControl()
    cmd.group_mix = 1
    cmd.controls[4] = -1
    cmd.controls[5] = -1
    n.cmd_pub.publish(cmd)
    print("Release mechanism is LOCKED !")
    

    while not rospy.is_shutdown() :

        #print("start to Calculate")
        n.bomb_status_pub.publish(bomb_status)
        
        rospy.wait_for_service('lla2enu')
        lla2enu = rospy.ServiceProxy('lla2enu', WgsConversion)
        # distance to target p

        dist2p = lla2enu(lla=(n.gps.latitude,n.gps.longitude,n.gps.altitude),ref_lla = (ref_lat,ref_long,ref_alt))
        (x,y,u) = dist2p.enu
        #print ("\t\tE: ",x,"\tN: ",y,"\tU: ",u)

        d = n.D * rot_pos(x,y,np.deg2rad(90))/np.sqrt(x**2+y**2)
        td=np.array([x, y])*(1+n.C*n.calDropDistance(n.alt,n.vel) /np.sqrt(x**2+y**2) ) #a way point to represent the direction of target
        #print("DistCal=",n.calDropDistance(n.alt,n.vel))
        
        if np.dot([-x,-y,0],n.gps_vel)/np.sqrt(n.gps_vel[0]**2+n.gps_vel[1]**2)  > np.dot([-x+d[0],-y+d[1],0],n.gps_vel)/np.sqrt((-x+d[0])**2+(-y+d[1])**2):
            #print("YEEEE~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
            #print("sqrt(x**2+y**2)=",np.sqrt(x**2+y**2))
            if n.calDropDistance(n.alt,n.vel) > np.sqrt(x**2+y**2):
                if n.mode == "OFFBOARD" :
                    print("~~~~~launch~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                    print("DistNow",np.sqrt(x**2+y**2),"DistCal=",n.calDropDistance(n.alt,n.vel))
                    
                    n.bomb_release = True
                    
                    bomb_status.data = n.bomb_release
                    n.bomb_status_pub.publish(bomb_status)
                    
                    
                    #cmd = ActuatorControl()
                    cmd.group_mix = 1
                    cmd.controls[4] = 1 
                    cmd.controls[5] = 1
                    n.cmd_pub.publish(cmd)
                    print("Bomb released ....")
                    break

    while n.bomb_release is True:
        n.bomb_status_pub.publish(bomb_status)
    
    rospy.spin()
