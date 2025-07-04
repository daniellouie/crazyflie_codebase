import numpy as np
import time
import csv
import os

# ---------------------------------- for plotting the J_inv -----------------------------------------
LOG_DIR = os.path.expanduser("~/crazyfly_ws/I_Joc_values")     
os.makedirs(LOG_DIR, exist_ok=True)  

class Cluster_new:

    # TODO : decide between initalizing variables in the constructor or in the class
    def __init__(self):

        # Class Variables 
        # TODO : consider moving this to instance variables for more efficiency
        self.R_cmd_ours = np.zeros(8)   # position commands of individual drones (x, y, z, yaw) in our frame
        self.R_cur_ours = np.zeros(8)   # current position of individual drones (x, y, z, yaw) in our frame
        self.R_dot = np.zeros(8) # commanded velocities of individual drones (x_dot, y_dot, z_dot, yaw_dot)
        self.C_des = np.zeros(8) # desired cluster position (xc, yc, zc, alpha, beta, phi1, phi2, p)
        self.C_cur = np.zeros(8) # current cluster position (xc, yc, zc, alpha, beta, phi1, phi2, p)
        self.C_cmd = np.zeros(8) # commanded cluster position (xc, yc, zc, alpha, beta, phi1, phi2, p)
        self.C_err = np.zeros(8) # cluster position error (xc, yc, zc)
        self.C_dot = np.zeros(8) # cluster velocity (xc_dot, yc_dot, zc_dot, alpha_dot, beta_dot, phi1_dot, phi2_dot, p_dot)

        self.cluster_gains = [
            1.0,  #X gains:
            1.0,  #Y gains:
            1.0,  #Z gains:
            1.0,  #alpha gains:
            1.0,  #beta gains:
            1.0,  #phi1 gains: always 0
            1.0,  #phi2 gains: always 0
            1.0,  #p gains:
        ]


        self.dt = 1 # constant term to convert velocity to position
        # dt = 0.6 May 9th


        #-------------------------- PLOTTING THE INVERSE JACOBIAN FOR LOOKING AT INPUT COMMANDS ---------------------------------------#
        self.cluster_dot = []
        #-----------------------------------------------------------------------------------------------------------------------------#


    def update(self):
        # current positions are called in the clusterController to get optitrack data
        self.forwardKinematics()  #convert drone positions to cluster space variables
        self.getClusterError()  #get cluster position error

        self.clusterPController()  #calculate cluster velocity using P controller
        # NOTE : not currently using Inverse Kinematic approach
        # self.clusterPControllerKinematic()  #calculate commanded cluster position using P controller for Inverse Kinematics

        self.clusterToDrones()  # uses Inverse Jacobian function convert cluster velocity to commanded drone velocities
        self.velocityToPos()  #convert commanded drone velocities to commanded drone positions

        # NOTE : not currently using Inverse Kinematic approach
        # self.calculateInverseKinematics()  #calculate commanded drone positions from desired cluster position
    
    # this function gets the positions from optitrack through the clusterController
    # Function is called in the clusterController
    def updatePositions(self, cur_position_cf1, cur_position_cf2):
        self.R_cur_ours[0:3] = cur_position_cf1  # Update Drone 1 position
        self.R_cur_ours[4:7] = cur_position_cf2  # Update Drone 2 position


    # This function converts individual drone positions into cluster space variables
    # Input: array of D1 and D2 positions [x, y, z, yaw] for each drone, 8 variables total
    # Output: array of cluster space variables [xc yc zc alpha beta phi1 phi2 p]
    # TODO : review and test this function (possibly seperate into externsal file)
    def forwardKinematics(self, R = 1):    
        if isinstance(R, int):
            # decompose positions and orientations
            x1, y1, z1, theta1 = self.R_cur_ours[0:4]
            x2, y2, z2, theta2 = self.R_cur_ours[4:8]
            # print("x1, y1, z1, theta1", x1, y1, z1, theta1)
            # print("x2, y2, z2, theta2", x2, y2, z2, theta2)
        else:
            x1, y1, z1, theta1 = R[0:4]
            x2, y2, z2, theta2 = R[4:8]



        # Global frame unit vectors
        xglobal = np.array([1, 0, 0])
        yglobal = np.array([0, 1, 0])
        zglobal = np.array([0, 0, 1])

        # Cluster position (centroid of the two drones)
        xc = (x1 + x2) / 2
        yc = (y1 + y2) / 2
        zc = (z1 + z2) / 2


        alpha, beta, phi1, phi2 = 0, 0, 0, 0

        # ---------------- NEW ---------------- #
        #NOTE: This now in the OUR New local variables
        gamma = 0 # not used in this implementation
        alpha = np.arctan2((x2-x1), (z2-z1))
        beta = np.arctan2((y2-y1), np.sqrt((z2-z1)**2 + (x2-x1)**2))
        phi1 = alpha - theta1
        phi2 = alpha - theta2


        # # Yaw corrected for the cluster frame (heading)
        # NOTE : not needed for current implementation, yaw of individual drones is controlled in individual PIDs
        # phi1 = theta1 - alpha
        # phi2 = theta2 - alpha

        # Cdistance between drones
        p = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)   

        # Cluster space variables
        if isinstance(R, int):
            self.C_cur = [xc, yc, zc, alpha, beta, phi1, phi2, p]
            return self.C_cur  
        else:
            return np.array([xc, yc, zc, alpha, beta, phi1, phi2, p])
        
    # This function gets the position error of cluster
    # Input: C_des, C_cur
    # Output: C_err
    # NOTE : needs to use all 8 variables for inverseJacobian
    def getClusterError(self):
        self.C_err = np.array(self.C_des) - np.array(self.C_cur)
        return self.C_err

    # This function calculates Cluster velocity (C_dot) using a P Controller
    # Input: C_err, cluster_k_p
    # Output: C_dot
    # TODO : needs tuning for p value, possibly use different gains for each axis?
    def clusterPController(self):
        # Calculate cluster velocity
        self.C_dot = self.cluster_gains * self.C_err
        return self.C_dot

    # This function calculates the inverse Jacobian for two drones
    # # Input: C_cur
    # Output: J_inv
    # TODO : review and test this function
    def calculateInverseJacobian(self):
        # Decompose the cluster state vector
        xC, yC, zC, alpha, beta, phi1, phi2, p = self.C_cur
        
        # NOTE: theta_C is alpha and Beta is beta
        
        # Define the rows of the inverse Jacobian matrix (using element-wise operations, equivalent of ./ in MATLAB)

        # --------------------------------------------------------------- ROWS OF J_INV --------------------------------------------------------------- #
        x1dot = [1, 0, 0, (-p/2)  * np.cos(alpha) * np.cos(beta), (p/2) * np.sin(alpha) * np.sin(beta), 0, 0, (-1/2) * np.cos(beta) * np.sin(alpha)]
        y1dot = [0, 1, 0, 0, (-p/2) * np.cos(beta), 0, 0, (-1/2) * np.sin(beta)]
        z1dot = [0, 0, 1, (p/2)*np.sin(alpha)*np.cos(beta), (p/2)*np.cos(alpha)*np.sin(beta), 0, 0, (-p/2) * np.cos(alpha) * np.cos(beta)]
        theta1dot = [0, 0, 0, 1, 0, -1, 0, 0]
        x2dot = [1, 0, 0, (p/2) * np.cos(alpha) * np.cos(beta), (-p/2) * np.sin(alpha) * np.sin(beta), 0, 0, (1/2) * np.cos(beta) * np.sin(alpha)]
        y2dot = [0, 1, 0, 0, (p/2) * np.cos(beta), 0, 0, (1/2) * np.sin(beta)]
        z2dot = [0, 0, 1, (-p/2) * np.sin(alpha) * np.cos(beta), (-p/2)*np.cos(alpha)*np.sin(beta), 0, 0, (1/2) * np.cos(alpha) * np.cos(beta)]
        theta2dot = [0, 0, 0, 1, 0, 0, -1, 0]
        # --------------------------------------------------------------------------------------------------------------------------------------------- #
        # Combine rows into the inverse Jacobian matrix
        J_inv = np.array([x1dot, y1dot, z1dot, theta1dot, x2dot, y2dot, z2dot, theta2dot])

        return J_inv
    



    # This function uses the resulting Inverse Jacobian (J_inv) to convert cluster velocity (C_dot) to commanded drone velocities (R_dot)
    # Input: C_dot, C_cur
    # Output: R_dot
    # TODO : review and test this function
    def clusterToDrones(self):
        # Calculate the commanded velocities for each drone
        J_inv = self.calculateInverseJacobian()
        self.R_dot = np.dot(J_inv, self.C_dot)
        
        #-------------------------------------------- PLOTTING THE INVERSE JACOBIAN VALUES ------------------------------------------------------------ #
        self.cluster_dot.append([
            time.time(),  # wall-clock seconds
                              
            #####  values for all axes  #####
            *self.R_dot[0:3],  # (x,y,z)_dot of drone 1
            *self.R_dot[4:7],  # (x,y,z)_dot of drone 2
            #####                     ######

            # self.R_dot[0],   # x_dot of drone 1
            # self.R_dot[4]])  # x_dot of drone 2
])
        

        return self.R_dot
    


    def dump_cluster_dot(self, fname="cluster_dot.csv"):
        path = os.path.join(LOG_DIR, fname)             # <— file ends up here
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(       # header row
                ["time_s",
                "x1dot","y1dot","z1dot",
                "x2dot","y2dot","z2dot"])
            w.writerows(self.cluster_dot)
        print(f"[Cluster] x-velocity log written to {path}")
    
    # --------------------------------------------------------------------------------------------------------------------------------------------- #
    



    # This function converts from the global cluster frame to the local drone frame
    # to account for difference in cluster orientation and drone orientation
    # TODO : need to take in theta1 and theta2, currently not processed bc they are in quaternion form (putting on the backburner for now)
    # def droneRotation(self):
    #     Rz = np.array([
    #         [np.cos(theta), np.sin(theta), 0],
    #         [np.sin(theta), -np.cos(theta), 0],
    #         [0, 0, 1]
    #     ])
    #     global_velocity_vector = np.array(global_velocity_vector[]

    
    # This function converts commanded drone velocities (R_dot) to commanded drone positions (R_cmd) using a dt constant
    # Input: R_dot, dt
    # Output: R_cmd
    # TODO : need to tune the dt contant for better performance
    def velocityToPos(self):
        self.R_cmd_ours = self.R_cur_ours + (self.R_dot * self.dt)
        return self.R_cmd_ours