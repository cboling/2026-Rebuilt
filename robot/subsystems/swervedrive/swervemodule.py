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

from phoenix6.swerve.swerve_module import SwerveModule as PhoenixSwerveModule
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.units import meters, meters_per_second

from lib_6107.subsystems.pykit.swervedrive_io import SwerveModuleIO
from pykit.logger import Logger
from util.logtracer import LogTracer


class SwerveModule(SwerveModuleIO):
    """
    The main work of the SwerveModule is done in generated code from the CTRE
    Tuner-X utility. This class wraps that generated code with what we need to
    support for AdvantageScope (via pykit).
    """

    def __init__(self, module: PhoenixSwerveModule, name: str):
        super().__init__(name)

        self.name = name
        self.module = module

        #self.io = SwerveModuleIO(name)
        self.inputs = SwerveModuleIO.SwerveModuleIOInputs()
        self.previous_position = SwerveModulePosition()

        self._wheel_radius: meters = 0.4  # TODO: Pull from tuner constants
        self._min_wheel_linear_velocity: meters_per_second = 0.002  # TODO: Pull from tuner constants

    def updateInputs(self):
        if self.inputs:
            pass

        # # TODO: FOLLOWING was from differential drive. change to SwerveDrive support
        # #
        # # TODO: Once we support PYKIT, add 'getPosition...' methods and decoreate them
        # #       with the @autolog_output. See pykit example
        # self._inputs.updateInputs(self._inputs)
        # Logger.processInputs("Drive", self._inputs)
        #
        # # Call into gyro explicitly since it is not a separate subsyste, but
        # # self.gyroIO.updateInputs(self._gyroInputs)   TODO: This is in Gyro subclass
        #
        # # Logger.processInputs("Drive/Gyro", self._gyroInputs)
        #
        # if self._gyroInputs.connected:
        #     self.rawGyroRotation = self._gyroInputs.yawPosition
        # else:
        #     twist = self.kinematics.toTwist2d(
        #         self.getLeftPosition() - self.lastLeftPosition,
        #         self.getRightPosition() - self.lastRightPosition,
        #     )
        #     self.rawGyroRotation = self.rawGyroRotation + Rotation2d(twist.dtheta)
        #
        # self.lastLeftPosition = self.getLeftPosition()
        # self.lastRightPosition = self.getRightPosition()
        #
        # self.poseEstimator.update(
        #     self.rawGyroRotation, self.getLeftPosition(), self.getRightPosition()
        # )

    def perform(self) -> None:
        LogTracer.resetOuter(f"SwerveModule/{self.name}")
        self.previous_position = self.getPosition()
        LogTracer.record("GetPosition")
        self.updateInputs(self.inputs)

        LogTracer.record("UpdateInputs")
        Logger.processInputs(f"Drive/Module{self.name}", self.inputs)
        LogTracer.record("ProcessInputs")
        LogTracer.recordTotal()
        #
        # TODO: The above is not called yet
        #

    def getSwerveAngle(self) -> Rotation2d:
        return Rotation2d(self.inputs.turn_position)

    def setSwerveAngle(self, swerve_angle: Rotation2d) -> None:
        raise NotImplemented("TODO") # ".io.setSwerveAngle(swerve_angle)

    def getSwerveEncoderAngle(self) -> Rotation2d:
        return Rotation2d(self.inputs.turn_absolute_position)

    def setSwerveAngleTarget(self, swerve_angle_target: Rotation2d) -> None:
        raise NotImplemented("TODO") # self.io.setSwerveAngleTarget(swerve_angle_target)

    def getWheelLinearVelocity(self) -> float:
        return self.inputs.drive_velocity * self._wheel_radius

    def getWheelTotalPosition(self) -> float:
        return self.inputs.drive_position * self._wheel_radius

    def setWheelLinearVelocityTarget(self, wheel_linear_velocity_target: float) -> None:
        raise NotImplemented("TODO")  #  self.io.setWheelLinearVelocityTarget(wheel_linear_velocity_target / self._wheel_radius)

    def reset(self) -> None:
        self.setSwerveAngle(self.getSwerveEncoderAngle())

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.getWheelTotalPosition(), self.getSwerveAngle())

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.getWheelLinearVelocity(),
                                 self.getSwerveAngle())

    def applyState(self, state: SwerveModuleState) -> None:
        state.optimize(self.getSwerveAngle())

        self.setWheelLinearVelocityTarget(state.speed)

        # prevent unnecessary movement for what would otherwise not move the robot
        if abs(state.speed) >= self._min_wheel_linear_velocity:
            self.setSwerveAngleTarget(state.angle)
