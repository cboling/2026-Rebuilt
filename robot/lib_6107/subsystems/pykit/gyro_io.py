# ------------------------------------------------------------------------ #
#      o-o      o                o                                         #
#     /         |                |                                         #
#    O     o  o O-o  o-o o-o     |  oo o--o o-o o-o                        #
#     \    |  | |  | |-' |   \   o | | |  |  /   /                         #
#      o-o o--O o-o  o-o o    o-o  o-o-o--O o-o o-o                        #
#             |                           |                                #
#          o--o                        o--o                                #
#                        o--o      o         o                             #
#                        |   |     |         |  o                          #
#                        O-Oo  o-o O-o  o-o -o-    o-o o-o                 #
#                        |  \  | | |  | | |  |  | |     \                  #
#                        o   o o-o o-o  o-o  o  |  o-o o-o                 #
#                                                                          #
#    Jemison High School - Huntsville Alabama                              #
# ------------------------------------------------------------------------ #

from dataclasses import dataclass

from wpimath.units import radians

from pykit.autolog import autolog


class GyroIO:
    @autolog
    @dataclass
    class GyroIOInputs:
        connected: bool = False
        yaw: radians = 0.0
        yaw_rate: float = 0.0

    def updateInputs(self, inputs: GyroIOInputs) -> None:
        pass

    def set_yaw(self, yaw_rad: radians) -> None:
        """
        Set the gyro yaw.

        Args:
            yaw_rad (float): The yaw in radians.
        """
