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

import logging

from pathplannerlib.auto import RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController

from wpilib import DriverStation
from wpimath.kinematics import ChassisSpeeds
from typing import Optional

from commands2 import cmd
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.events import EventTrigger
from pathplannerlib.logging import PathPlannerLogging
from wpilib import SendableChooser

from pykit.logger import Logger

from subsystems.swervedrive.drivesubsystem import DriveSubsystem

from lib_6107.commands.drivetrain.arcade_drive import ArcadeDrive
from lib_6107.commands.drivetrain.aimtodirection import AimToDirection
from lib_6107.commands.drivetrain.approach_tag import ApproachTag
from lib_6107.commands.drivetrain.gotopoint import GoToPoint
from lib_6107.commands.drivetrain.swervetopoint import SwerveToPoint, SwerveMove

logger = logging.getLogger(__name__)


def configure_auto_builder(drivetrain: DriveSubsystem,
                           default_command: Optional[str] = "") -> SendableChooser:
    config = RobotConfig.fromGUISettings()
    AutoBuilder.configure(
        lambda: drivetrain.get_state().pose,    # Supplier of current robot pose
        drivetrain.reset_pose,                  # Consumer for seeding pose against auto
        lambda: drivetrain.get_state().speeds,  # Supplier of current robot speeds

        # Consumer of ChassisSpeeds and feedforwards to drive the robot
        lambda speeds, feedforwards: drivetrain.set_control(
            drivetrain.apply_robot_speeds
            .with_speeds(ChassisSpeeds.discretize(speeds, 0.020))
            .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
            .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
        ),
        PPHolonomicDriveController(
            # PID constants for translation
            PIDConstants(10.0, 0.0, 0.0),
            # PID constants for rotation
            PIDConstants(7.0, 0.0, 0.0)
        ),
        config,
        # Assume the path needs to be flipped for Red vs Blue, this is normally the case
        lambda: (DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
        drivetrain  # Subsystem for requirements
    )
    # PathPlanner and AdvantageScope integration
    PathPlannerLogging.setLogActivePathCallback(lambda path: Logger.recordOutput("Odometry/Trajectory",
                                                                                 path))
    PathPlannerLogging.setLogTargetPoseCallback(lambda pose: Logger.recordOutput("Odometry/TrajectorySetpoint",
                                                                                 pose))
    # TODO: More SysId below
    # self.sysid = SysIdRoutine(SysIdRoutine.Config(1, 7, 10,
    #                                               lambda state: Logger.recordOutput("Drive/SysIdState",
    #                                                                                 sysIdStateToStr(state)),),
    #                           SysIdRoutine.Mechanism((lambda volts: self.runOpenLoop(volts, volts)),
    #                                                  (lambda: None), self))    # Register all the library 'named' commands we may wish to use

    return register_commands_and_triggers(drivetrain, default_command)

    # TODO: AdvantageScope support below

def register_commands_and_triggers(drivetrain: DriveSubsystem,
                                   default_command: Optional[str] = "") -> SendableChooser:
    # Register Named Commands
    commands = [
        ArcadeDrive,
        AimToDirection,
        ApproachTag,
        GoToPoint,
        SwerveToPoint,
        SwerveMove,
    ]
    for item in commands:
        item.pathplanner_register(drivetrain)

    # Now all the triggers

    EventTrigger("Collect Fuel").whileTrue(cmd.PrintCommand("running intake"))

    # Load in any Autonomous Commands into the chooser

    return AutoBuilder.buildAutoChooser(default_command)
