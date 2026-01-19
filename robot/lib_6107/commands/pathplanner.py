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

from typing import Optional

from commands2 import cmd
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.events import EventTrigger
from wpilib import SendableChooser

from lib_6107.commands.drivetrain.arcade_drive import ArcadeDrive

# from lib_6107.commands.drivetrain.aimtodirection import AimToDirection
# from lib_6107.commands.drivetrain.approach_tag import ApproachTag
# from lib_6107.commands.drivetrain.gotopoint import GoToPoint
# from lib_6107.commands.drivetrain.swervetopoint import SwerveToPoint

def register_commands_and_triggers(drivetrain: 'DriveSubsystem',
                                   default_command: Optional[str] = "") -> SendableChooser:
    # Register Named Commands
    commands = [
        ArcadeDrive,
    ]
    for item in commands:
        item.pathplanner_register(drivetrain)

    # Now all the triggers

    EventTrigger("Collect Fuel").whileTrue(cmd.PrintCommand("running intake"))

    # Load in any Autonomous Commands into the chooser
    # return AutoBuilder.buildAutoChooser(default_command)
    return None
