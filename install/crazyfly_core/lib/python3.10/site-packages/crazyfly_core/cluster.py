import numpy as np

class Cluster:

    # TODO : decide between initalizing variables in the constructor or in the class
    def __init__(self):

        # Class Variables 
        # TODO : consider moving this to instance variables for more efficiency
        self.R_cmd = np.zeros(8) # array for position commands of individual drones (x, y, z, yaw)
        self.R_cur = np.zeros(8) # array for current position of individual drones (x, y, z, yaw)
        self.R_dot = np.zeros(8) # array for commanded velocities of individual drones (x_dot, y_dot, z_dot, yaw_dot)
        self.C_des = np.zeros(8) # array for desired cluster position (xc, yc, zc, alpha, beta, phi1, phi2, p)
        self.C_cur = np.zeros(8) # array for current cluster position (xc, yc, zc, alpha, beta, phi1, phi2, p)
        self.C_cmd = np.zeros(8) # array for commanded cluster position (xc, yc, zc, alpha, beta, phi1, phi2, p)
        self.C_err = np.zeros(3) # array for cluster position error (xc, yc, zc)
        self.C_dot = np.zeros(8) # array for cluster velocity (xc_dot, yc_dot, zc_dot, alpha_dot, beta_dot, phi1_dot, phi2_dot, p_dot)

        self.cluster_k_p = 0.1 # proportional gain for cluster controller
        self.dt = 6 # constant term to convert velocity to position


    def frameOursToAnne(self, our_frame):
        transformation_matrix = np.array([
            [0, 1, 0],  # x -> Y
            [0, 0, 1],  # y -> Z
            [1, 0, 0]   # z -> X
        ])
        
        our_drone1_pos = our_frame[:3]
        our_drone2_pos = our_frame[4:7]

        anne_drone1_pos = np.dot(our_drone1_pos, transformation_matrix)
        # print("anne_drone1_pos", anne_drone1_pos)
        anne_drone2_pos = np.dot(our_drone2_pos, transformation_matrix)
        # print("anne_drone2_pos", anne_drone2_pos)

        anne_frame = np.concatenate((anne_drone1_pos, [our_frame[3]], anne_drone2_pos, [our_frame[7]]))
        
        return anne_frame
    

    def frameAnneToOurs(self, anne_frame):
        transformation_matrix = np.array([
            [0, 0, 1],  # X -> z
            [1, 0, 0],  # Y -> x
            [0, 1, 0]   # Z -> y
        ])

        anne_drone1_pos = anne_frame[:3]
        anne_drone2_pos = anne_frame[4:7]

        our_drone1_pos = np.dot(anne_drone1_pos, transformation_matrix)
        # print("our_drone1_pos", our_drone1_pos)
        our_drone2_pos = np.dot(anne_drone2_pos, transformation_matrix)
        # print("our_drone2_pos", our_drone2_pos)

        our_frame = np.concatenate((our_drone1_pos, [anne_frame[3]], our_drone2_pos, [anne_frame[7]]))        
        return our_frame
    

    def update(self):
        # self.getPosition()  #get updated position of drones from Optitrack
        self.forwardKinematics()  #convert drone positions to cluster space variables
        self.getClusterError()  #get cluster position error
        self.clusterPController()  #calculate cluster velocity using P controller
        self.clusterToDrones()  # uses Forward Kinematic function convert cluster velocity to commanded drone velocities
        self.velocityToPos()  #convert commanded drone velocities to commanded drone positions

    # This function takes in current positions of drones from Optitrack mo-cap
    # Input: none
    # Output: updates R_cur 
    # TODO : implement this function, connect to Optitrack
    def getPosition(self):
        # self.R_cur = [1, 1, 1, 0, 2, 2, 2, 0] # this is a placeholder for testing
        return self.R_cur
    
    # this function gets the positions from optitrack through the clusterController
    def updatePositions(self, cur_position_cf1, cur_position_cf2):
        self.R_cur[0:3] = cur_position_cf1  # Update Drone 1 position
        self.R_cur[4:7] = cur_position_cf2  # Update Drone 2 position


    # This function converts individual drone positions into cluster space variables
    # Input: array of D1 and D2 positions [x, y, z, yaw] for each drone, 8 variables total
    # Output: array of cluster space variables [xc yc zc alpha beta phi1 phi2 p]
    # TODO : review and test this function (possibly seperate into external file)
    def forwardKinematics(self):    
        # TODO : frame transformation Ours to Anne's
        self.R_cur = self.frameOursToAnne(self.R_cur) # convert to Anne's frame

        # decompose positions and orientations
        x1, y1, z1, theta1 = self.R_cur[0:4]
        x2, y2, z2, theta2 = self.R_cur[4:8]
        # print("x1, y1, z1, theta1", x1, y1, z1, theta1)
        # print("x2, y2, z2, theta2", x2, y2, z2, theta2)

        # Global frame unit vectors
        xglobal = np.array([1, 0, 0])
        yglobal = np.array([0, 1, 0])
        zglobal = np.array([0, 0, 1])

        # Cluster position (centroid of the two drones)
        xc = (x1 + x2) / 2
        yc = (y1 + y2) / 2
        zc = (z1 + z2) / 2


        alpha, beta, phi1, phi2 = 0, 0, 0, 0

        # # Vectors pointing to Drone 1
        # y_vec = np.array([x1 - xc, y1 - yc, z1 - zc])
        # x_vec = np.cross(y_vec, zglobal)
        # z_vec = np.cross(x_vec, y_vec)

        # # Normalize vectors (using element-wise division, equivalent of ./ in MATLAB)
        # y_vec_norm = y_vec / np.linalg.norm(y_vec)
        # x_vec_norm = x_vec / np.linalg.norm(x_vec)
        # z_vec_norm = z_vec / np.linalg.norm(z_vec)

        # # Rotation matrix
        # rot = np.array([
        #     [np.dot(x_vec_norm, yglobal), np.dot(y_vec_norm, yglobal), np.dot(z_vec_norm, yglobal)],
        #     [np.dot(x_vec_norm, xglobal), np.dot(y_vec_norm, xglobal), np.dot(z_vec_norm, xglobal)],
        #     [np.dot(x_vec_norm, zglobal), np.dot(y_vec_norm, zglobal), np.dot(z_vec_norm, zglobal)]
        # ])

        # # Rotation matrix angles
        # alpha = np.arctan2(rot[1,0], rot[0,0])
        # beta = np.arctan2(rot[2,1], rot[2,2])
        # gamma = 0 # not used in this implementation

        # # Yaw corrected for the cluster frame (heading)
        # phi1 = theta1 - alpha
        # phi2 = theta2 - alpha

        # Cdistance between drones
        p = np.sqrt((x1 - x2)**2 + (y1 - y2)**2 + (z1 - z2)**2)   

        # Cluster space variables
        self.C_cur = [xc, yc, zc, alpha, beta, phi1, phi2, p]
        return self.C_cur  #this is the equivalent of Anne's K array in the paper
        
    # This function gets the position error of cluster
    # Input: C_des, C_cur
    # Output: C_err
    # TODO : implement this function
    # NOTE : should this calculate the error in all 8 dimensions or just the first 3? (needs to be 8 for Jinv part)
    def getClusterError(self):
        # self.C_err = np.array(self.C_des[:3]) - np.array(self.C_cur[:3])
        self.C_err = np.array(self.C_des) - np.array(self.C_cur)

        return self.C_err

    # This function calculates Cluster velocity (C_dot) using a P Controller
    # Input: C_err, cluster_k_p
    # Output: C_dot
    # TODO : implement this function, do we need a seperate P term for each axis?
    def clusterPController(self):
        # Calculate cluster velocity
        self.C_dot = self.cluster_k_p * self.C_err
        return self.C_dot

    # This function calculates the inverse Jacobian for two drones
    # # Input: C_cur
    # Output: J_inv
    # TODO : review and test this function
    def calculateInverseJacobian(self):
        # Decompose the cluster state vector
        xC, yC, zC, alpha, beta, phi1, phi2, p = self.C_cur

        # Define the rows of the inverse Jacobian matrix (using element-wise operations, equivalent of ./ in MATLAB)
        x1dot = [1, 0, 0, (-1/2) * p * np.cos(alpha) * np.cos(beta), (1/2) * p * np.sin(alpha) * np.sin(beta), 0, 0, (-1/2) * np.cos(beta) * np.sin(alpha)]
        y1dot = [0, 1, 0, (-1/2) * p * np.cos(beta) * np.sin(alpha), (-1/2) * p * np.cos(alpha) * np.sin(beta), 0, 0, (1/2) * np.cos(alpha) * np.cos(beta)]
        z1dot = [0, 0, 1, 0, (1/2) * p * np.cos(beta), 0, 0, (1/2) * np.sin(beta)]
        theta1dot = [0, 0, 0, 1, 0, 1, 0, 0]

        x2dot = [1, 0, 0, (1/2) * p * np.cos(alpha) * np.cos(beta), (-1/2) * p * np.sin(alpha) * np.sin(beta), 0, 0, (1/2) * np.cos(beta) * np.sin(alpha)]
        y2dot = [0, 1, 0, (1/2) * p * np.cos(beta) * np.sin(alpha), (1/2) * p * np.cos(alpha) * np.sin(beta), 0, 0, (-1/2) * np.cos(alpha) * np.cos(beta)]
        z2dot = [0, 0, 1, 0, (-1/2) * p * np.cos(beta), 0, 0, (-1/2) * np.sin(beta)]
        theta2dot = [0, 0, 0, 1, 0, 0, 1, 0]

        # Combine rows into the inverse Jacobian matrix
        J_inv = np.array([x1dot, y1dot, z1dot, theta1dot, x2dot, y2dot, z2dot, theta2dot])

        return J_inv
    
    # This function uses the resulting Inverse Jacobian (J_inv) to convert cluster velocity (C_dot) to commanded drone velocities (R_dot)
    # Input: C_dot, C_cur
    # Output: R_dot
    # TODO : review and test this function
    # NOTE : function wants C_dot to be 8x1, does that make sense?
    def clusterToDrones(self):
        # Calculate the commanded velocities for each drone
        J_inv = self.calculateInverseJacobian()
        self.R_dot = np.dot(J_inv, self.C_dot)
        return self.R_dot
    
    # This function converts commanded drone velocities (R_dot) to commanded drone positions (R_cmd) using a dt constant
    # Input: R_dot, dt
    # Output: R_cmd
    # TODO : review and test this function
    def velocityToPos(self):
        self.R_cmd = self.R_cur + (self.R_dot * self.dt)

        # TODO:frame transformation Anne's to Ours
        self.R_cmd = self.frameAnneToOurs(self.R_cmd) # convert to our frame
        # print("R_cmd_transformed (actual)", self.R_cmd)

        return self.R_cmd
    

    







