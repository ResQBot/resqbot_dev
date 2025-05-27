import roboticstoolbox as rtb
from spatialmath.base import *
from spatialmath import SE3
import os
from ament_index_python import get_package_share_directory



# Robot Static Kinematics
lotti_arm = rtb.Robot()

lotti_package = get_package_share_directory('lotti_control')

lotti_urdf = os.path.join(lotti_package, 'description/urdf', 'Lotti.urdf.xacro')

lotti_arm.URDF_read(lotti_urdf)
print(lotti_arm)


# Calculating Forward Kinematics
fk_tran_m = lotti_arm.fkine([1.911, 0.8322, -0.4118, -1.557, -2.733, 0.8391, -0.5577])
print(fk_tran_m)


# Calculating Inverse Kinematics
point = SE3(0.0, 0.7,0.5)
ik_tran_m = lotti_arm.ikine_LM(point)
print(ik_tran_m)