import numpy as np
from cluster import Cluster  # Import your Cluster class

# Initialize the Cluster object
cluster = Cluster()

# Placeholder input for drone positions (x, y, z, yaw for each drone)
# Example: Drone 1 at (1, 1, 1, 0), Drone 2 at (2, 2, 2, 0)
cluster.R_cur = np.array([1,0,1,0,2,0,1,0])

# Set a desired cluster position (example values)\
# NOTE : C_des needs to converted to Anne's frame (done)
C_des_ours = np.array([1.5,1,2,0,0,0,0,1])
cluster.C_des = C_des_ours

# Step-by-step testing
# print("Testing frameOursToAnne...")
# R_cur_transformed = cluster.frameOursToAnne(cluster.R_cur)
# print("R_cur_transformed (transformed positions):", R_cur_transformed)

print("Testing forwardKinematics...")
C_cur = cluster.forwardKinematics()
print("C_cur (current cluster state):", C_cur)

print("\nTesting getClusterError...")
C_err = cluster.getClusterError()
print("C_err (cluster position error):", C_err)

print("\nTesting clusterPController...")
C_dot = cluster.clusterPController()
print("C_dot (cluster velocity):", C_dot)

print("\nTesting calculateInverseJacobian...")
J_inv = cluster.calculateInverseJacobian()
print("J_inv (inverse Jacobian):\n", J_inv)

print("\nTesting clusterToDrones...")
R_dot = cluster.clusterToDrones()
print("R_dot (commanded drone velocities):", R_dot)

print("\nTesting velocityToPos...")
R_cmd = cluster.velocityToPos()
print("R_cmd (commanded drone positions):", R_cmd)

# make sure we only transform it once