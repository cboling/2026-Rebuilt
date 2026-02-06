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

from wpimath.geometry import Rotation2d
from wpimath.units import amperes, radians, radians_per_second, volts

from pykit.autolog import autolog

"""
SwerveDriveIO provides  drive I/O to provide log information
for AdvantageScope replay and simulation.
"""


@dataclass
class SwerveModuleConfigParams:
    swerveEncoderOffset: float
    swerveEncoderID: int
    driveMotorID: int
    driveMotorInverted: bool
    driveGearing: float
    steerMotorID: int
    steerMotorInverted: bool
    steerGearing: float
    canbus: str = ""

    # pylint:disable-next=too-many-arguments, too-many-positional-arguments
    def __init__(
            self,
            driveMotorID: int,
            driveMotorInverted: bool,
            driveGearing: float,
            steerMotorID: int,
            steerMotorInverted: bool,
            steerGearing: float,
            swerveEncoderID: int,
            swerveEncoderOffset: float,
            canbus: str = "",
    ) -> None:
        self.driveMotorID = driveMotorID
        self.driveMotorInverted = driveMotorInverted
        self.driveGearing = driveGearing
        self.steerMotorID = steerMotorID
        self.steerMotorInverted = steerMotorInverted
        self.steerGearing = steerGearing
        self.swerveEncoderID = swerveEncoderID
        self.swerveEncoderOffset = swerveEncoderOffset
        self.canbus = canbus


class SwerveModuleIO:
    @autolog
    @dataclass
    class SwerveModuleIOInputs:
        drive_connected: bool = False
        steer_connected: bool = False
        encoder_connected: bool = False

        drive_position: radians = 0.0  # rad
        drive_velocity: radians_per_second = 0.0  # rad / sec
        drive_applied: volts = 0.0  # volts
        drive_supply_current: amperes = 0.0  # amps
        drive_torque_current: amperes = 0.0  # amps

        turn_position: radians = 0.0  # rad
        turn_velocity: radians_per_second = 0.0  # rad / sec
        turn_applied: volts = 0.0  # volts
        turn_supply_current: amperes = 0.0  # amps
        turn_torque_current: amperes = 0.0  # amps

        turn_absolute_position: float = 0.0  # rad

    def __init__(self, name: str) -> None:
        self.name = name

    def updateInputs(self, inputs: SwerveModuleIOInputs) -> None:
        """Update the swerve module I/O inputs.

        Args:
            inputs (SwerveModuleIOInputs): The swerve module I/O inputs to update.
        """
        pass

    def setSwerveAngle(self, swerveAngle: Rotation2d) -> None:
        """Set the swerve module angle.

        Args:
            swerveAngle (Rotation2d): The desired swerve module angle.
        """

    def setWheelLinearVelocityTarget(self, wheelLinearVelocityTarget: float) -> None:
        """Set the swerve module wheel linear velocity.

        Args:
            wheelLinearVelocity_m_per_s (float): The desired wheel linear velocity in meters per second.
        """

    def setSwerveAngleTarget(self, swerveAngleTarget: Rotation2d) -> None:
        """Set the swerve module angle target.

        Args:
            swerveAngleTarget (Rotation2d): The desired swerve module angle target.
        """
