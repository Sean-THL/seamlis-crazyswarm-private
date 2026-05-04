import argparse
import atexit

import numpy as np

from . import genericJoystick
from std_msgs.msg import Bool
import rospy
import cvxpy as cp


# Building the parser in a separate function allows sphinx-argparse to
# auto-generate the documentation for the command-line flags.
def build_argparser(parent_parsers=[]):
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        parents=parent_parsers
    )
    parser.add_argument("--sim", help="Run using simulation.", action="store_true")

    group = parser.add_argument_group("Simulation-only", "")
    group.add_argument("--vis", help="Visualization backend.", choices=['mpl', 'vispy', 'null'], default="mpl")
    group.add_argument("--dt", help="Duration of seconds between rendered visualization frames.", type=float, default=0.1)
    group.add_argument("--writecsv", help="Enable CSV output.", action="store_true")
    group.add_argument("--disturbance", help="Simulate Gaussian-distributed disturbance when using cmdVelocityWorld.", type=float, default=0.0)
    group.add_argument("--maxvel", help="Limit simulated velocity (meters/sec).", type=float, default=np.inf)
    group.add_argument("--video", help="Video output path.", type=str)

    return parser


class Crazyswarm:
    def __init__(self, crazyflies_yaml=None, parent_parser=None, args=None):
        self.status = True
        self.triggered = False
        rospy.Subscriber("/swarm/status", Bool, self.status_update)    

        if parent_parser is not None:
            parents = [parent_parser]
        else:
            parents = []
        parser = build_argparser(parents)
        if isinstance(args, str):
            args = args.split()
        args, unknown = parser.parse_known_args(args)

        if crazyflies_yaml is None:
            crazyflies_yaml = "../launch/crazyflies.yaml"
        if crazyflies_yaml.endswith(".yaml"):
            crazyflies_yaml = open(crazyflies_yaml, 'r').read()

        if args.sim:
            from .crazyflieSim import TimeHelper, CrazyflieServer
            self.timeHelper = TimeHelper(args.vis, args.dt, args.writecsv, disturbanceSize=args.disturbance, maxVel=args.maxvel, videopath=args.video)
            self.allcfs = CrazyflieServer(self.timeHelper, crazyflies_yaml)
            atexit.register(self.timeHelper._atexit)
        else:
            from .crazyflie import TimeHelper, CrazyflieServer
            self.allcfs = CrazyflieServer(crazyflies_yaml)
            self.timeHelper = TimeHelper()
            if args.writecsv:
                print("WARNING: writecsv argument ignored! This is only available in simulation.")
            if args.video:
                print("WARNING: video argument ignored! This is only available in simulation.")

        self.input = genericJoystick.Joystick(self.timeHelper)


        # Saves the initial positions of the drones
        self.init_pos = np.array([drone.position()[:2] for drone in self.allcfs.crazyflies])
        self.init_pos_z = np.array([drone.position() for drone in self.allcfs.crazyflies])
        self.n = len(self.allcfs.crazyflies)

        # Land the drones when the node crashes
        rospy.on_shutdown(self.emergency_land)


    # Brings the drones back to their initial positions while performing collision avoidance
    def return_initial_controller(self):
        
        if self.n>1:
            ################################## CBF-QP Controller ####################################
            D_MIN = 0.4
            num_constraints = int((self.n-1)*(self.n)/2)
            u1 = cp.Variable((self.n*2,1))
            u1_des = cp.Parameter((self.n*2,1),value = np.zeros((self.n*2,1)) )
            A1 = cp.Parameter((num_constraints,self.n*2),value=np.zeros((num_constraints,self.n*2)))
            b1 = cp.Parameter((num_constraints,1),value=np.zeros((num_constraints,1)))
            const1 = [A1 @ u1 >= b1]
            objective1 = cp.Minimize( cp.sum_squares( u1 - u1_des  ))
            cbf_controller = cp.Problem( objective1, const1 )
            #########################################################################################
            current_pos = np.array([drone.position()[:2] for drone in self.allcfs.crazyflies])
            # Solves the central CBF-QP until the drones are close to their initial positions while enforcing collision avoidance
            time_original = self.timeHelper.time()
            while np.any(np.linalg.norm(current_pos-self.init_pos, axis=1) > 0.35):
                for i in range(self.n):
                    vector = (self.init_pos[i] - current_pos[i]).reshape(-1,1)
                    if np.linalg.norm(vector) <= 0.35:
                        vector = np.zeros((2,1))
                    else:
                        vector= vector/np.linalg.norm(vector)/2
                    u1_des.value[2*i:2*i+2] = vector
                    for j in range(i+1, self.n):
                        h =  np.linalg.norm(current_pos[i] - current_pos[j])**2 - D_MIN**2 
                        dh_dxi = 2*(current_pos[i] - current_pos[j]).T
                        A1.value[i,2*i:2*i+2] =  dh_dxi[:]
                        A1.value[i,2*j:2*j+2] = -dh_dxi[:]
                        b1.value[i] = -1.5*h
                cbf_controller.solve(solver = "GUROBI")
                # Implements the control input from CBF-QP
                for i in range(self.n):
                    vec = u1.value[2*i:2*i+2].reshape(-1,2)[0]
                    if np.linalg.norm(vec)< 0.01:
                        vec = np.zeros((2,))
                    self.allcfs.crazyflies[i].cmdVelocityWorld(np.append(vec,0), yawRate=0)
                self.timeHelper.sleep(0.01)
                current_pos = np.array([drone.position()[:2] for drone in self.allcfs.crazyflies])
                
                new_time = self.timeHelper.time()
                if np.abs(time_original-new_time)>10:
                    break
            # Stops the drones 
            for i in range(self.n):
                self.allcfs.crazyflies[i].cmdVelocityWorld(np.zeros(3,), yawRate=0)

            self.emergency_land_all()
            self.land_all()
        else:
            current_pos = np.array([drone.position()[:2] for drone in self.allcfs.crazyflies])
            dir = self.init_pos - current_pos
            dir = np.append(dir, 0)
            dir = dir/np.linalg.norm(dir)
            self.allcfs.crazyflies[0].cmdVelocityWorld(dir, yawRate=0)
            self.timeHelper.sleep(5)

            self.emergency_land_all()
            self.land_all()

        
    # Drives the drones back to their initial positions and lands the drones
    def emergency_land(self):
        # if not self.status:
        #     print("Emergency not triggered")
        #     return 1 
        # self.return_initial_controller()

        Z_SPEED = 0.75 # m/s
        LAND_HEIGHT = 0.04 #m
        max_duration = 0.0
        for cf in self.allcfs.crazyflies:
            z = cf.position()[2]
            duration = z / Z_SPEED + 2
            max_duration = max(max_duration, duration)
            cf.land(targetHeight=LAND_HEIGHT, duration=duration)
        self.timeHelper.sleep(max_duration)


    def emergency_land_all(self):
        Z_SPEED = 0.75 # m/s
        timeHelper = self.timeHelper
        self.status = False
        current_height = np.array([drone.position()[2] for drone in self.allcfs.crazyflies])
        while np.any(current_height>0.2):
            for cf in self.allcfs.crazyflies:
                cf.cmdVelocityWorld(np.array([0.0,0.0,-0.75]), yawRate=0)
            timeHelper.sleep(0.01)
            current_height = np.array([drone.position()[2] for drone in self.allcfs.crazyflies])


    def land_all(self):
        Z_SPEED = 0.75 # m/s
        LAND_HEIGHT = 0.04 #m
        timeHelper = self.timeHelper
        max_duration = 0.0
        self.status = False
        for cf in self.allcfs.crazyflies:
            z = cf.position()[2]
            duration = z / Z_SPEED + 1
            max_duration = max(max_duration, duration)
            cf.land(targetHeight=LAND_HEIGHT, duration=duration)
        timeHelper.sleep(max_duration)


    # The Crazyswarm constantly listens to the published status
    def status_update(self,msg):
        self.status = msg.data
        if not self.status and not self.triggered:
            self.triggered = True
            self.return_initial_controller()
            self.timeHelper.sleep(2)
            self.land_all()

