#!/usr/bin/env python

import numpy as np
from roboticstoolbox.robot.Robot import Robot
from spatialmath import SE3


class Lotti(Robot):

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "/home/paul/resqbot_dev/control_ws/src/lotti_control/description/urdf/Lotti_arm.urdf.xacro"
        )

        super().__init__(
            links,
            name=name,
            manufacturer="Res.Q Bots"
            gripper_links=links[7],
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.grippers[0].tool = SE3(0, 0, 0)

        self.qdlim = np.array(
            [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100]
        )

        self.rest = np.array([0, -1.1, 2.0, 0, 0, 0])

        self.addconfiguration("rest", self.rest)


if __name__ == "__main__":  # pragma nocover

    r = Lotti()

    r.rest

    for link in r.grippers[0].links:
        print(link)