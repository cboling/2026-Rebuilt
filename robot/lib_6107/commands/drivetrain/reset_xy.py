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
# From Gene Panov's (Team 714) CommandRevSwerve project (and FRC Python videos)

from typing import Optional

from wpimath.geometry import Rotation2d, Pose2d, Translation2d
from wpimath.units import degrees
from lib_6107.commands.command import BaseCommand

class ResetXY(BaseCommand):

    def __init__(self, drivetrain: 'DriveSubsystem',
                 x: Optional[float | int] = 0,
                 y: Optional[float | int] = 0,
                 heading: Optional[degrees] = 0.0):
        """
        Reset the starting (X, Y) and heading (in degrees) of the robot to where they should be.
        :param x: X
        :param y: X
        :param heading: heading (for example: 0 = "North" of the field, 180 = "South" of the field)
        :param drivetrain: drivetrain on which the (X, Y, heading) should be set
        """
        super().__init__(drivetrain)

        self.position = Pose2d(Translation2d(x, y), Rotation2d.fromDegrees(heading))

    def initialize(self):
        super().initialize()

        self._drivetrain.pose = self.position

    def isFinished(self) -> bool:
        return True  # this is an instant command, it finishes right after it initialized

    def execute(self):
        """
        nothing to do here, this is an instant command
        """

    def end(self, interrupted: bool):
        """
        nothing to do here, this is an instant command
        """
        super().end(interrupted)
