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

from typing import Callable, List, Tuple

from pykit.logger import Logger
from wpilib import RobotBase
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition

from constants import DriveKinematics
from util.logtracer import LogTracer

# from constants.turret import kTurretLocation
# from constants.auto import kAutoDistanceTolerance, kAutoRotationTolerance
# from constants.field import kEndgameDuration, kShiftDuration

ModulePositionType = Tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]


class RobotState:
    headingOffset: Rotation2d = Rotation2d()
    robotHeading: Rotation2d = Rotation2d()
    turretRotation: Rotation2d = Rotation2d()

    modulePositions: ModulePositionType = (SwerveModulePosition(), SwerveModulePosition(),
                                           SwerveModulePosition(), SwerveModulePosition())

    # estimator: TurretedRobotPoseEstimator = TurretedRobotPoseEstimator(
    #     kDriveKinematics, Rotation2d(), modulePositions, Pose2d(), (0.1, 0.1, 0.1)
    # )
    odometry: SwerveDrive4Odometry = SwerveDrive4Odometry(DriveKinematics, Rotation2d(),
                                                          modulePositions, Pose2d())
    simResetPoseConsumers: List[Callable[[Pose2d], None]] = []
    simPoseReceiverConsumers: List[Callable[[], Pose2d]] = []

    robotFieldVelocity: ChassisSpeeds = ChassisSpeeds()

    targetAutonomousStartingLocation: Pose2d = Pose2d()

    @classmethod
    def setAutonomousStartingLogation(cls, location: Pose2d):
        cls.targetAutonomousStartingLocation = location

    @classmethod
    def periodic(cls, heading: Rotation2d, headingTimestamp: float, robotYawVelocity: float,
                 fieldRelativeRobotVelocity: ChassisSpeeds, modulePositions: ModulePositionType) -> None:

        LogTracer.resetOuter("RobotState")

        cls.robotHeading = heading
        cls.modulePositions = modulePositions
        # cls.odometry.update(heading, modulePositions)

        cls.robotFieldVelocity = fieldRelativeRobotVelocity

        LogTracer.record("OdometryUpdate")
        # cls.estimator.addOdometryMeasurement(
        #     OdometryObservation(modulePositions, heading, headingTimestamp)
        # )
        # cls.estimator.addTurretMeasurement(
        #     TurretObservation(turretRotation, headingTimestamp)
        # )
        LogTracer.record("EstimatorUpdate")

        estimatedFieldPose = cls.getPose()
        Logger.recordOutput("Robot/Pose/EstimatorPose", estimatedFieldPose)
        Logger.recordOutput("Robot/Pose/OdometryPose", cls.odometry.getPose())
        Logger.recordOutput("Robot/TurretRotation", cls.turretRotation)
        Logger.recordOutput("Robot/Heading", cls.robotHeading)
        Logger.recordOutput("Robot/HeadingVelocity", robotYawVelocity)
        Logger.recordOutput("Robot/Velocity", fieldRelativeRobotVelocity)
        Logger.recordOutput("Robot/HeadingOffset", cls.headingOffset)

        autoPositionDelta = estimatedFieldPose - cls.targetAutonomousStartingLocation

        Logger.recordOutput("Auto/PositionOffset", autoPositionDelta)
        Logger.recordOutput("Auto/PositionCorrect",
                            autoPositionDelta.translation().norm() < kAutoDistanceTolerance)
        Logger.recordOutput("Auto/RotationCorrect",
                            abs(autoPositionDelta.rotation().radians()) < kAutoRotationTolerance)
        # Logger.recordOutput("Auto/StartingPose", cls.targetAutonomousStartingLocation)
        # Logger.recordOutput("Game/WonAuto", cls.didWinAuto())
        # Logger.recordOutput("Game/HubActive", cls.hubActive())

        if not RobotBase.isReal():
            Logger.recordOutput("Robot/SimPose", cls.getSimPose())
            Logger.recordOutput("Robot/SimTurretPose", cls.getSimTurretPose())

        LogTracer.recordTotal()

    @classmethod
    def getPose(cls) -> Pose2d:
        return cls.estimator.estimatedPose

    @classmethod
    def getRotation(cls) -> Rotation2d:
        return cls.getPose().rotation()

    @classmethod
    def resetPose(cls, pose: Pose2d = Pose2d()) -> None:
        cls.headingOffset = cls.robotHeading - pose.rotation()
        cls.odometry.resetPosition(cls.robotHeading, cls.modulePositions, pose)
        cls.estimator.resetPosition(cls.robotHeading, cls.modulePositions, pose)

        if RobotBase.isSimulation() and not Logger.isReplay():
            cls.resetSimPose(pose)
    #
    # @classmethod
    # def resetSimPose(cls, pose: Pose2d):
    #     if len(cls.simResetPoseConsumers) > 0:
    #         for consumer in cls.simResetPoseConsumers:
    #             consumer(pose)
    #         return
    #     print("This is not supposed to happen")

    # @classmethod
    # def registerSimPoseResetConsumer(cls, consumer: Callable[[Pose2d], None]) -> None:
    #     cls.simResetPoseConsumers.append(consumer)

    # @classmethod
    # def getSimPose(cls) -> Pose2d:
    #     if len(cls.simPoseReceiverConsumers) == 1:
    #         return cls.simPoseReceiverConsumers[0]()
    #     print("This is not supposed to happen")
    #     return cls.getPose()

    # @classmethod
    # def getSimTurretPose(cls) -> Pose3d:
    #     return (
    #         pose3dFrom2d(cls.getSimPose())
    #         + kTurretLocation
    #         + Transform3d(0, 0, 0, Rotation3d(0, 0, cls.turretRotation.radians()))
    #     )
    #
    # @classmethod
    # def registerSimPoseRecieverConsumer(cls, consumer: Callable[[], Pose2d]) -> None:
    #     cls.simPoseReceiverConsumers.append(consumer)
