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
import math
from typing import Callable
from typing import Tuple, Optional, List, Dict, Union, Any, Generator

import ntcore
import robotpy_apriltag as apriltag
from commands2 import Command, Subsystem, InstantCommand
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder, RobotConfig
from pathplannerlib.commands import DriveFeedforwards
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from phoenix6 import SignalLogger, swerve, units, utils
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from wpilib import DriverStation, Notifier, RobotController
from wpilib import SmartDashboard, Field2d, RobotBase, Timer
from wpilib.sysid import SysIdRoutineLog
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.geometry import Transform2d, Transform3d, Translation3d, Rotation3d, \
    Pose3d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from wpimath.kinematics import SwerveModuleState, SwerveDrive4Kinematics, \
    SwerveModulePosition
from wpimath.units import degrees, inchesToMeters

from frc_2026.constants import USE_PYKIT
from frc_2026.field import RED_TEST_POSE, BLUE_TEST_POSE, FIELD_Y_SIZE, FIELD_X_SIZE
from frc_2026.generated.tuner_constants import TunerSwerveDrivetrain
from frc_2026.subsystems import constants
from frc_2026.subsystems.swervedrive import swerveutils
from frc_2026.subsystems.swervedrive.constants import DriveConstants
from lib_6107.subsystems.pykit.swervedrive_io import SwerveDriveIO

try:
    import navx

    NAVX_SUPPORTED = True
except ImportError:
    NAVX_SUPPORTED = False

# TODO: This value needs to be tested. Perform the following on a real robot
#
# Measuring Overshoot
# Implement a Control Loop:
#   You cannot simply apply constant power until the target angle is reached, as the robot needs time to decelerate
#   and will inevitably overshoot. A simple Proportional (P) loop is the standard starting point for FRC teams.
#   The motor power is made proportional to the difference between the target angle and the current angle.
#
#      Formula (simplified P-loop): motorPower = (targetAngle - currentAngle) * kP
#      kP is a constant you tune to get the desired performance.
#
# Log Data:
#   Use your FRC development environment (e.g., WPILib) to log the robot's current gyro angle and the target
#   angle to a file or SmartDashboard/Shuffleboard.
#
# Perform a Test Turn:
#   Command your robot to turn to a specific, significant angle (e.g., 90 degrees) using the P-loop,
#   and log the data during the process.
#
# Analyze the Data:
#   After the test, view the logged data in a graph or spreadsheet.
# Target Angle:
#   The desired final angle (e.g., 90 degrees).
#
# Peak Angle:
#   The maximum angle the robot reaches during the turn before it starts correcting back towards the target.
#
# Calculate Overshoot:
#   The difference between the peak angle and the target angle is the overshoot.
#
# Overshoot = Peak Angle - Target Angle
#
# Correcting Overshoot
#
# The primary method for reducing overshoot is tuning your control loop.
#
# Adjust kP:
#   If your robot consistently overshoots significantly, your kP value is likely too high. Lowering it
#   will make the turn slower but more accurate.
#
# Add Derivative (D) control:
#   Implementing a full PID loop can help. The derivative term (kD) dampens the system by applying a
#   counter-force based on how fast the error is changing (i.e., the robot's turn rate), which helps
#   slow the robot down as it approaches the target.
#
# Slow Down Turns:
#   As a simple fix, reducing the maximum motor power or speed used for turns will also reduce overshoot,
#   though it makes the robot slower overall.
#
# Ensure Proper Calibration:
#   Make sure the gyro is stationary during its initial calibration phase (when the robot code starts)
#   to minimize drift and baseline errors.
#
GYRO_OVERSHOOT_FRACTION = -3.25 / 360
# ^^ our gyro didn't overshoot, it "undershot" by 0.1 degrees in a 360 degree turn

SwerveModuleStats = Tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]

logger = logging.getLogger(__name__)


@autologgable_output
class DriveSubsystem(Subsystem, TunerSwerveDrivetrain):
    _SIM_LOOP_PERIOD: units.second = 0.004  # 4 ms

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """Red alliance sees forward as 180 degrees (toward blue alliance wall)"""

    def __init__(self, consts, modules, container: 'RobotContainer',
                 max_speed_scale_factor: Optional[Callable[[None], float]] = None,
                 **kwargs: Optional[Dict[str, Any]]) -> None:

        # super().__init__(hardware.TalonFX, hardware.TalonFX, hardware.CANcoder, consts, modules)
        Subsystem.__init__(self)
        TunerSwerveDrivetrain.__init__(self, consts, modules)

        if max_speed_scale_factor is not None:
            assert callable(max_speed_scale_factor)

        self._container = container
        self._robot = container.robot

        # self.kinematics: SwerveDrive4Kinematics = DriveConstants.DRIVE_KINEMATICS  # our swerve drive kinematics

        # Init the Auto chooser.  PathPlanner init will fill in our choices
        self.autoChooser: LoggedDashboardChooser[Command] = LoggedDashboardChooser("Auto Choices")

        # Camera/localizer defaults
        self.front_camera = None
        self.vision_odometry = False
        self.field_relative = False

        cameras: Dict[str, Any] = kwargs.get("Cameras")
        if cameras is not None and "Front" in cameras:
            self.front_camera = cameras["Front"]["Camera"]

            localizer = cameras["Front"].get("Localizer")
            if localizer:
                self.vision_odometry = True
                self.field_relative = True
                self.localizer = localizer

        # TODO: Make sure and tie this into the SmartDashboard chooser
        # TODO: Make sure and tie this into the SmartDashboard chooser
        # TODO: Make sure and tie this into the SmartDashboard chooser
        # TODO: Make sure and tie this into the SmartDashboard chooser
        self.maxSpeedScaleFactor: Optional[Callable[[], float]] = max_speed_scale_factor

        # self.gyroOvershootFraction = 0.0
        # if not TimedCommandRobot.isSimulation():
        #     self.gyroOvershootFraction = GYRO_OVERSHOOT_FRACTION
        #
        # # Create MAXSwerveModules
        # self.frontLeft = MAXSwerveModule(
        #     DeviceID.DRIVETRAIN_LEFT_FRONT_DRIVING_ID,
        #     DeviceID.DRIVETRAIN_LEFT_FRONT_TURNING_ID,
        #     DriveConstants.FRONT_LEFT_ANGULAR_OFFSET,
        #     driveMotorInverted=DriveConstants.FRONT_LEFT_DRIVE_MOTOR_INVERTED,
        #     turnMotorInverted=DriveConstants.FRONT_LEFT_TURNING_MOTOR_INVERTED,
        #     motorControllerType=SparkMax,
        #     encoder_analog_port=DeviceID.DRIVETRAIN_LEFT_FRONT_ENCODER_ID,
        #     label="lf")
        #
        # self.frontRight = MAXSwerveModule(
        #     DeviceID.DRIVETRAIN_RIGHT_FRONT_DRIVING_ID,
        #     DeviceID.DRIVETRAIN_RIGHT_FRONT_TURNING_ID,
        #     DriveConstants.FRONT_RIGHT_ANGULAR_OFFSET,
        #     driveMotorInverted=DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_INVERTED,
        #     turnMotorInverted=DriveConstants.FRONT_RIGHT_TURNING_MOTOR_INVERTED,
        #     motorControllerType=SparkMax,
        #     encoder_analog_port=DeviceID.DRIVETRAIN_RIGHT_FRONT_ENCODER_ID,
        #     label="rf")
        #
        # self.rearLeft = MAXSwerveModule(
        #     DeviceID.DRIVETRAIN_LEFT_REAR_DRIVING_ID,
        #     DeviceID.DRIVETRAIN_LEFT_REAR_TURNING_ID,
        #     DriveConstants.REAR_LEFT_ANGULAR_OFFSET,
        #     driveMotorInverted=DriveConstants.REAR_LEFT_DRIVE_MOTOR_INVERTED,
        #     turnMotorInverted=DriveConstants.REAR_LEFT_TURNING_MOTOR_INVERTED,
        #     motorControllerType=SparkMax,
        #     encoder_analog_port=DeviceID.DRIVETRAIN_LEFT_REAR_ENCODER_ID,
        #     label="lb")
        #
        # self.rearRight = MAXSwerveModule(
        #     DeviceID.DRIVETRAIN_RIGHT_REAR_DRIVING_ID,
        #     DeviceID.DRIVETRAIN_RIGHT_REAR_TURNING_ID,
        #     DriveConstants.REAR_RIGHT_ANGULAR_OFFSET,
        #     driveMotorInverted=DriveConstants.REAR_RIGHT_DRIVE_MOTOR_INVERTED,
        #     turnMotorInverted=DriveConstants.REAR_RIGHT_TURNING_MOTOR_INVERTED,
        #     motorControllerType=SparkMax,
        #     encoder_analog_port=DeviceID.DRIVETRAIN_RIGHT_REAR_ENCODER_ID,
        #     label="rb")
        #
        # self.swerve_modules: List[MAXSwerveModule] = [self.frontLeft, self.frontRight, self.rearLeft, self.rearRight]

        # The gyro/IMU sensor

        # self._gyro: Optional[Gyro] = None

        # if DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_NAVX and NAVX_SUPPORTED:
        #     self._gyro = NavX(DriveConstants.GYRO_REVERSED)
        #
        # elif DriveConstants.GYRO_TYPE == DriveConstants.GYRO_TYPE_PIGEON2:
        #     # Note: Default pigeon2 config has compass disabled. We want it that way as well.
        #     self._gyro = Pigeon2(DeviceID.GYRO_DEVICE_ID, DriveConstants.GYRO_REVERSED,
        #                          kwargs["pykit"]["Update Frequency"])
        #
        # self._gyro.initialize()
        # self._gyroInputs = GyroIO.GyroIOInputs()

        # TODO: Support slew rate and make adjustable. There is a parameter to 'drive()'
        #       called rate limit that currently uses the rotation & magnitude limiter
        # Slew rate filter variables and limiters for controlling lateral acceleration
        self.magLimiter = SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE)
        self.rotLimiter = SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE)

        # self.currentTranslationDir = 0.0
        # self.currentTranslationMag = 0.0
        # self.xSpeedDelivered = 0.0
        # self.ySpeedDelivered = 0.0
        # self.rotDelivered = 0.0
        # self.prevTime = Timer.getFPGATimestamp()

        # Pykit support
        self.inputs = SwerveDriveIO.DriveIOInputs()

        # The next attributes are set depending on if vision is unsupported for tracking the robot pose
        self.odometry = None
        self._network_table_inst = None

        if not self.vision_odometry or RobotBase.isSimulation():
            # The robots movements are commanded based on the fixed coordinate system of the competition field
            self.field_relative = True

            # # Odometry class for tracking robot pose
            # self.odometry = SwerveDrive4Odometry(DriveConstants.DRIVE_KINEMATICS,
            #                                      Rotation2d(),
            #                                      (self.frontLeft.getPosition(),
            #                                       self.frontRight.getPosition(),
            #                                       self.rearLeft.getPosition(),
            #                                       self.rearRight.getPosition()))

        else:
            # The robots movements are commanded based on the robot's own orientation
            self.field_relative = False
            self._init_vision_odometry()

            # self.field = self.quest_field
            self._robot.field = self.quest_field

        initial_pose = self._alliance_change(container.is_red_alliance,
                                             container.alliance_location)

        # positions = tuple(self.get_module_positions())

        # #   TODO: If not done already, register for an alliance change callback. If something
        # #         changes, can
        # self.pose_estimator = SwerveDrive4PoseEstimator(DriveConstants.DRIVE_KINEMATICS,
        #                                                 Rotation2d.fromDegrees(self._gyro.angle),
        #                                                 positions,
        #                                                 initialPose=initial_pose)

        self.odometryHeadingOffset = Rotation2d(0)

        # # # TODO: SUPPORT PATHPLANNER
        # #
        # #   TODO: If not done already, register for an alliance change callback. If something
        # #         changes, can we update our  'pose_estimator above' and the settings below?
        # #
        # #   TODO: Is alliance settings in a match already in driverstation before we start and all this only matters during simulation?
        #
        # robot_config = RobotConfig.fromGUISettings()
        #
        # # TODO: What do we need here with pathplanner and assuming pykit is also supported. Also if
        # #       we want the PPLTV Controller, should we use the AutoConstants or DriveConstants.
        # # controller = apriltag.k_pathplanner_holonomic_controller
        # controller = PPLTVController(1 / kwargs["pykit"]["Update Frequency"],
        #                              DriveConstants.MAX_SPEED_METERS_PER_SECOND)
        #
        # # # TODO: Validate what get_relative_speeds is suppose to return in the actual call.
        # # #       it looks like it is suppose to be 4 values and not a list of 4 values (ModuleStates)
        # # AutoBuilder.configure(self.get_pose,
        # #                       self.resetOdometry,
        # #                       self.get_relative_speeds,
        # #                       self.drive_robot_relative,
        # #                       controller,
        # #                       robot_config,
        # #                       lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
        # #                       self)
        # #
        # # PathPlannerLogging.setLogActivePathCallback(lambda path: Logger.recordOutput("Odometry/Trajectory",
        # #                                                                              path))
        # # PathPlannerLogging.setLogTargetPoseCallback(lambda pose: Logger.recordOutput("Odometry/TrajectorySetpoint",
        # #                                                                              pose))
        # # TODO: More SysId below
        # # self.sysid = SysIdRoutine(SysIdRoutine.Config(1, 7, 10,
        # #                                               lambda state: Logger.recordOutput("Drive/SysIdState",
        # #                                                                                 sysIdStateToStr(state)),),
        # #                           SysIdRoutine.Mechanism((lambda volts: self.runOpenLoop(volts, volts)),
        # #                                                  (lambda: None), self))
        #
        # # Register for any changes in alliance before the match starts
        container.register_alliance_change_callback(self._alliance_change)

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""

        # Swerve request to apply during path following
        self._apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()

        # Swerve requests to apply during SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                )
                                          and None,
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing translation. This is used to find PID gains for the drive motors."""

        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                )
                                          and None,
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing steer. This is used to find PID gains for the steer motors."""

        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                )
                                          and None,
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                                   # output is actually radians per second, but SysId only supports "volts"
                                   self.set_control(
                                       self._rotation_characterization.with_rotational_rate(output)
                                   ),
                                   # also log the requested output for SysId
                                   SignalLogger.write_double("Rotational_Rate", output),
                               )
                               and None,
                lambda log: None,
                self,
            ),
        )
        """
        SysId routine for characterizing rotation.
        This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        """

        self._sys_id_routine_to_apply = self._sys_id_routine_translation
        """The SysId routine to test"""

        if utils.is_simulation():
            self._start_sim_thread()

        self._configure_auto_builder()

    def _configure_auto_builder(self):
        config = RobotConfig.fromGUISettings()
        AutoBuilder.configure(
            lambda: self.get_state().pose,  # Supplier of current robot pose
            self.reset_pose,  # Consumer for seeding pose against auto
            lambda: self.get_state().speeds,  # Supplier of current robot speeds
            # Consumer of ChassisSpeeds and feedforwards to drive the robot
            lambda speeds, feedforwards: self.set_control(
                self._apply_robot_speeds
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
            self  # Subsystem for requirements
        )

    def apply_request(
            self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Quasistatic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Dynamic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.dynamic(direction)

    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        # Run simulation at a faster rate so PID gains behave more reasonably
        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

    def add_vision_measurement(
            self,
            vision_robot_pose: Pose2d,
            timestamp: units.second,
            vision_measurement_std_devs: tuple[float, float, float] | None = None,
    ):
        """
        Adds a vision measurement to the Kalman Filter. This will correct the
        odometry pose estimate while still accounting for measurement noise.

        Note that the vision measurement standard deviations passed into this method
        will continue to apply to future measurements until a subsequent call to
        set_vision_measurement_std_devs or this method.

        :param vision_robot_pose:           The pose of the robot as measured by the vision camera.
        :type vision_robot_pose:            Pose2d
        :param timestamp:                   The timestamp of the vision measurement in seconds.
        :type timestamp:                    second
        :param vision_measurement_std_devs: Standard deviations of the vision pose measurement
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians.
        :type vision_measurement_std_devs:  tuple[float, float, float] | None
        """
        TunerSwerveDrivetrain.add_vision_measurement(
            self,
            vision_robot_pose,
            utils.fpga_to_current_time(timestamp),
            vision_measurement_std_devs
        )

    def sample_pose_at(self, timestamp: units.second) -> Pose2d | None:
        """
        Return the pose at a given timestamp, if the buffer is not empty.

        :param timestamp: The timestamp of the pose in seconds.
        :type timestamp: second
        :returns: The pose at the given timestamp (or None if the buffer is empty).
        :rtype: Pose2d | None
        """
        return TunerSwerveDrivetrain.sample_pose_at(self, utils.fpga_to_current_time(timestamp))

    @property
    def counter(self) -> int:
        return self._robot.counter

    @property
    def field(self) -> Field2d:
        return self._robot.field

    # @property
    # def gyro(self) -> Gyro:
    #     return self._gyro

    ###########################################
    # PathPlanner Support (TODO: NOT YET ENABLED - COMMENTED OUT IN INIT)
    def runClosedLoop(self, speeds: ChassisSpeeds,
                      _feedForwards: Optional[DriveFeedforwards] = None):
        wheelSpeeds = self.kinematics.toWheelSpeeds(speeds)
        self.runClosedLoopParameters(wheelSpeeds.left, wheelSpeeds.right)

    if USE_PYKIT:
        def runClosedLoopParameters(self, leftSpeed: float, rightSpeed: float):
            leftRadPerS = leftSpeed / driveconstants.kWheelRadius
            rightRadPerS = rightSpeed / driveconstants.kWheelRadius

            Logger.recordOutput("Drive/LeftSetpoint", leftRadPerS)
            Logger.recordOutput("Drive/RightSetpoint", rightRadPerS)

            leftFF = self.kS * sign(leftRadPerS) + self.kV * leftRadPerS
            rightFF = self.kS * sign(rightRadPerS) + self.kV * rightRadPerS

            self.io.setVelocity(leftRadPerS, rightRadPerS, leftFF, rightFF)

        def runOpenLoop(self, leftV: float, rightV: float) -> None:
            self.io.setVoltage(leftV, rightV)

        def sysIdQuasistatic(self, direction: SysIdRoutine.Direction):
            return self.sysid.quasistatic(direction)

        def sysIdDynamic(self, direction: SysIdRoutine.Direction):
            return self.sysid.dynamic(direction)

    def _init_vision_odometry_photoncam(self):
        # TODO: Below is code from team 2429 where they use vision to estimate position.
        # TODO: May be better derive a new class for this
        # 2024 - orphan the old odometry, now use the vision enabled version of odometry instead
        initialPose = Pose2d(constants.START_X,
                             constants.START_Y,
                             Rotation2d.fromDegrees(self.pigeon2.get_yaw().value))
        # get poses from NT
        self._network_table_inst = ntcore.NetworkTableInstance.getDefault()

        self.pi_subscriber_dicts: List[Dict[str, Union[ntcore.DoubleArraySubscriber, ntcore.DoubleSubscriber]]] = []
        for pi_name in constants.VisionConstants.k_pi_names:
            this_pi_subscriber_dict = {}
            this_pi_subscriber_dict.update({"robot_pose_info_subscriber": self._network_table_inst.getDoubleArrayTopic(
                f"vision/{pi_name}/robot_pose_info").subscribe([])})
            this_pi_subscriber_dict.update(
                {"wpinow_time_subscriber": self._network_table_inst.getDoubleTopic(
                    f"vision/{pi_name}/wpinow_time").subscribe(0)})
            self.pi_subscriber_dicts.append(this_pi_subscriber_dict)

        # photonvision camera setup
        self.use_photoncam = constants.USE_PHOTON_TAGS  # decide down below in periodic
        if self.use_photoncam:
            # TODO: Code still from team 2429 below
            from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy  # 2025 is first time for us

            # self.photon_name = "Arducam_OV9281_USB_Camera"
            # self.photon_name = "HD_Pro_Webcam_C920"
            self.photon_name = "Geniuscam"

            self.photoncam_arducam_a = PhotonCamera(self.photon_name)
            self.photoncam_target_subscriber = self._network_table_inst.getBooleanTopic(
                f'/photonvision/{self.photon_name}/hasTarget').subscribe(False)
            self.photoncam_latency_subscriber = self._network_table_inst.getDoubleTopic(
                f'/photonvision/{self.photon_name}/LatencyMillis').subscribe(0)

            # example is cam mounted facing forward, half a meter forward of center, half a meter up from center
            # robot_to_cam_example = wpimath.geometry.Transform3d(wpimath.geometry.Translation3d(0.5, 0.0, 0.5),
            #     wpimath.geometry.Rotation3d.fromDegrees(0.0, -30.0, 0.0),)
            robot_to_cam_arducam_a = Transform3d(
                Translation3d(inchesToMeters(10), inchesToMeters(7.75), 0.45),
                Rotation3d.fromDegrees(0.0, 0.0, math.radians(270)))

            robot_to_cam_arducam_a = Transform3d(
                Translation3d(inchesToMeters(0), inchesToMeters(0), 0.),
                Rotation3d.fromDegrees(0.0, -20, math.radians(0)))

            # geniuscam with standard convention - 10.5in x, -8in y, -111 degrees yaw
            robot_to_cam_arducam_a = Transform3d(
                Translation3d(inchesToMeters(0), inchesToMeters(0), 0.),
                Rotation3d.fromDegrees(0.0, 0, math.radians(0)))  # -111 from front if ccw +

            # todo - see if we can update the PoseStrategy based on if disabled, and use closest to current odometry when enabled
            # but MULTI_TAG_PNP_ON_COPROCESSOR probably does not help at all since we only see one tag at a time
            # TODO: we can set a fallback for multi_tag_pnp_on_coprocessor, we can make that lowest ambiguity or cloest to current odo
            self.photoncam_pose_est = PhotonPoseEstimator(
                apriltag.AprilTagFieldLayout.loadField(apriltag.AprilTagField.k2025ReefscapeWelded),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                self.photoncam_arducam_a,
                robot_to_cam_arducam_a)
            # default is above PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, but maybe PoseStrategy.LOWEST_AMBIGUITY is better
            self.photoncam_pose_est.primaryStrategy = PoseStrategy.LOWEST_AMBIGUITY

            # -----------   CJH simple apriltags  ------------
            # get poses from NT
            self.use_CJH_apriltags = constants.USE_CJH_TAGS  # down below we decide which one to use in the periodic method
            # lhack turned off 15:48 2/28/25 to test pathplanner wo tags first
            self._network_table_inst = ntcore.NetworkTableInstance.getDefault()
            # TODO - make this a loop with just the names
            self.arducam_back_pose_subscriber = self._network_table_inst.getDoubleArrayTopic(
                "/Cameras/ArducamBack/poses/tag1").subscribe([0] * 8)
            self.arducam_back_count_subscriber = self._network_table_inst.getDoubleTopic(
                "/Cameras/ArducamBack/tags/targets").subscribe(0)

            self.arducam_high_pose_subscriber = self._network_table_inst.getDoubleArrayTopic(
                "/Cameras/ArducamHigh/poses/tag1").subscribe([0] * 8)
            self.arducam_high_count_subscriber = self._network_table_inst.getDoubleTopic(
                "/Cameras/ArducamHigh/tags/targets").subscribe(0)

            self.genius_low_pose_subscriber = self._network_table_inst.getDoubleArrayTopic(
                "/Cameras/GeniusLow/poses/tag1").subscribe(
                [0] * 8)
            self.genius_low_count_subscriber = self._network_table_inst.getDoubleTopic(
                "/Cameras/GeniusLow/tags/targets").subscribe(0)

            self.logitech_reef_pose_subscriber = self._network_table_inst.getDoubleArrayTopic(
                "/Cameras/LogitechReef/poses/tag1").subscribe([0] * 8)
            self.logitech_reef_count_subscriber = self._network_table_inst.getDoubleTopic(
                "/Cameras/LogitechReef/tags/targets").subscribe(0)

            # set myself up for a zip later on
            self.pose_subscribers = [self.arducam_back_pose_subscriber, self.arducam_high_pose_subscriber,
                                     self.genius_low_pose_subscriber, self.logitech_reef_pose_subscriber]
            self.count_subscribers = [self.arducam_back_count_subscriber, self.arducam_high_count_subscriber,
                                      self.genius_low_count_subscriber, self.logitech_reef_count_subscriber]

            try:
                self._desired_april_tags = constants.VisionConstants.k_valid_tags
            except Exception:
                self._desired_april_tags = None  # TODO: Constants not yet defined

            # TODO - give me a list of six filters for the apriltags - smooth if we are not moving, else use reset each measurement
            # def tag_filter(window):
            #     return [wpimath.filter.LinearFilter.movingAverage(window) for _ in range(6) ]
            # window = 5
            # self.tag_motion_filters = [tag_filter(window) for _ in self.pose_subscribers]

            self.automated_path = None

            # QuestNav - eventually we will push all of this out to a class
            # TODO: This needs to be supported eventually (See Team 2429 robot for 2025)
            from helpers.questnav.questnav2 import QuestNav
            self.questnav = QuestNav()
            self.quest_to_robot = Transform2d(inchesToMeters(-8.35), inchesToMeters(-10.50),
                                              Rotation2d().fromDegrees(270))  # 10.50 -8.35
            # This is quest-centric coordinate. X is robot center position -8.35 inch as seen rom Quest. y is robot center -10.50 inches as seen from Quest
            # self.quest_to_robot = Transform2d(inchesToMeters(4), 0, Rotation2d().fromDegrees(0))
            self.quest_field = Field2d()
            self.quest_has_synched = False  # use this to check in disabled whether to update the quest with the robot odometry
            SmartDashboard.putBoolean('questnav_synched', self.quest_has_synched)
            self.use_quest = constants.USE_QUEST_ODOMETRY
            SmartDashboard.putBoolean('questnav_in_use', self.use_quest)

            # note - have Risaku standardize these with the rest of the putDatas
            SmartDashboard.putData('QuestResetOdometry',
                                          InstantCommand(lambda: self.quest_reset_odometry()).ignoringDisable(True))
            SmartDashboard.putData('QuestSyncOdometry',
                                          InstantCommand(lambda: self.quest_sync_odometry()).ignoringDisable(True))
            # SmartDashboard.putData('QuestUnSync', InstantCommand(lambda: self.quest_unsync_odometry()).ignoringDisable(True))
            SmartDashboard.putData('QuestEnableToggle',
                                          InstantCommand(lambda: self.quest_enabled_toggle()).ignoringDisable(True))
            SmartDashboard.putData('QuestSyncToggle',
                                          InstantCommand(lambda: self.quest_sync_toggle()).ignoringDisable(True))
            # end of init

    def _alliance_change(self, is_red: bool, location: int) -> None:
        """
        Change in alliance occurred before match started. If simulation is
        supported, then 'physics.py' handles this.
        """
        # TODO: Need to coordinate the value below with our default for red or blue and which location
        initial_pose = Pose2d(constants.START_X,
                              constants.START_Y,
                              Rotation2d.fromDegrees(self.pigeon2.get_yaw().value))

        if RobotBase.isSimulation():
            # Use test subsystem settings if simulation
            initial_pose = RED_TEST_POSE[location] if is_red else BLUE_TEST_POSE[location]
            self.resetOdometry(initial_pose)

        # TODO: Do we also need to update what we provide the pathplanner staring locaiont

        return initial_pose

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        SmartDashboard.putData("Field", self.field)
        # self._gyro.dashboard_initialize()

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        divisor = 10 if self._robot.isEnabled() else 20
        update_dash = self._robot.counter % divisor == 0

        # if update_dash:
        #     pose = self.get_pose()
        #
        #     SmartDashboard.putNumber("Drivetrain/x", pose.x)
        #     SmartDashboard.putNumber("Drivetrain/y", pose.y)
        #     SmartDashboard.putNumber("Drivetrain/heading", pose.rotation().degrees())
        #
        #     # self.gyro.dashboard_periodic()

    def configure_button_bindings(self, driver, shooter) -> None:
        """
        Configure the driver and shooter joystick controls here
        """
        if self.front_camera is not None:
            def turn_to_object() -> None:
                """
                This command is used to have the robot camera

                If you want the robot to slowly chase that object... replace the 'self.rotate'
                line below with: self.arcadeDrive(0.1, turn_speed)

                """
                x = self.front_camera.getX()
                turn_speed = -0.01 * x
                self.rotate(turn_speed)

            # TODO: Make the button assignment come from a constants.py file / list somewhere.
            #       so we can keep track what is assigned
            # from lib_6107.commands.camera.turn_to_object import turn_to_object
            #
            # button = self.driverController.button(XboxController.Button.kB)
            # button.whileTrue(RunCommand(turn_to_object, self))
            # button.onFalse(InstantCommand(lambda: self.drive(0, 0, 0)
            pass

        pass  # TODO: Add me

    def periodic(self) -> None:
        enabled = self._robot.isEnabled()
        log_it = self._robot.counter % 20 == 0 and enabled

        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

        # self._gyro.periodic(self._gyroInputs)

        if USE_PYKIT:
            # TODO: FOLLOWING was from differential drive. change to SwerveDrive support
            #
            # TODO: Once we support PYKIT, add 'getPosition...' methods and decoreate them
            #       with the @autolog_output. See pykit example
            self.io.updateInputs(self.inputs)
            self.gyroIO.updateInputs(self.gyroInputs)

            Logger.processInputs("Drive", self.inputs)
            Logger.processInputs("Drive/Gyro", self.gyroInputs)

            if self.gyroInputs.connected:
                self.rawGyroRotation = self.gyroInputs.yawPosition
            else:
                twist = self.kinematics.toTwist2d(
                    self.getLeftPosition() - self.lastLeftPosition,
                    self.getRightPosition() - self.lastRightPosition,
                )
                self.rawGyroRotation = self.rawGyroRotation + Rotation2d(twist.dtheta)

            self.lastLeftPosition = self.getLeftPosition()
            self.lastRightPosition = self.getRightPosition()

            self.poseEstimator.update(
                self.rawGyroRotation, self.getLeftPosition(), self.getRightPosition()
            )

        # fl_pos = self.frontLeft.getPosition()
        # fr_pos = self.frontRight.getPosition()
        # rl_pos = self.rearLeft.getPosition()
        # rr_pos = self.rearRight.getPosition()
        # heading = self._gyro.heading
        #
        # if log_it:
        #     logger.debug(
        #         f"updating odometry: heading: {heading}, fl_pos: {fl_pos}, fr_pos: {fr_pos}, rl_pos: {rl_pos}, rr_pos: {rr_pos}")
        #     logger.debug(f"pose before update: {self.get_pose()}")

        # Update the odometry in the periodic block
        if self.odometry:
            pose = self.odometry.update(heading, (fl_pos, fr_pos, rl_pos, rr_pos,))

            if log_it:
                logger.debug(
                    f"Drive periodic: gyro Heading: {self._gyro.heading}, x: {pose.x}, y: {pose.y}, rot: {pose.rotation().degrees()}")

            self.field.setRobotPose(pose)

        # Update SmartDashboard for this subsystem
        self.dashboard_periodic()

    def periodic_other(self) -> None:
        # TODO: Has team 2429 vision support. Keep until we can use it
        # send our current time to the dashboard
        ts = Timer.getFPGATimestamp()
        SmartDashboard.putNumber('_timestamp',
                                 ts)  # this one we actually do every time TODO - see if this is done by wpilib and use it instead

        # use this if we have a phononvision camera - which we don't as of 20250316
        if self.vision_odometry and RobotBase.isReal() and self.use_photoncam:  # sim complains if you don't set up a sim photoncam
            has_photontag = self.photoncam_target_subscriber.get()
            # has_photontag = self.photoncam_target_subscriber.get()
            # how do we get the time offset and standard deviation?

            if has_photontag:  # TODO - CHANGE ANGLE OF CAMERA MOUNTS
                result = self.photoncam_arducam_a.getLatestResult()
                cam_est_pose = self.photoncam_pose_est.update(result)
                # can also use result.hasTargets() instead of nt
                target = result.getBestTarget()
                if target is not None:  # not sure why it returns None sometimes when we have tags
                    # get id with target.fiducialId
                    # get % of camera with target.getArea() to get a sense of distance
                    try:
                        ambiguity = target.getPoseAmbiguity()
                    except AttributeError as e:
                        ambiguity = 999

                    latency = self.photoncam_latency_subscriber.get()
                    # if statements to test if we want to update using a tag
                    use_tag = constants.USE_PHOTON_TAGS  # can disable this in constants
                    # do not allow large jumps when enabled
                    delta_pos = Translation2d.distance(self.get_pose().translation(),
                                                       cam_est_pose.estimatedPose.translation().toTranslation2d())
                    use_tag = False if (
                            delta_pos > 1 and DriverStation.isEnabled()) else use_tag  # no big movements in odometry from tags
                    # limit a pose rotation to less than x degrees
                    delta_rot = math.fabs(
                        self.get_pose().rotation().degrees() - cam_est_pose.estimatedPose.rotation().angle_degrees)
                    use_tag = False if delta_rot > 10 and DriverStation.isEnabled() else use_tag
                    # TODO - ignore tags if we are moving too fast
                    use_tag = False if self._gyro.getRate() > 90 else use_tag  # no more than n degrees per second turning if using a tag
                    use_tag = False if latency > 100 else use_tag  # ignore stale tags

                    # TODO - filter out tags that are too far away from camera (different from pose itself too far away from robot)
                    # filter out tags with too much ambiguity - where ratio > 0.2 per docs
                    use_tag = False if ambiguity > 0.2 else use_tag

                    if use_tag:
                        self.pose_estimator.addVisionMeasurement(cam_est_pose.estimatedPose.toPose2d(), ts - latency,
                                                                 constants.DrivetrainConstants.k_pose_stdevs_large)
                # _ = self.photoncam_arducam_a.getAllUnreadResults()
            else:
                pass

            if self.counter % 10 == 0 and self.use_photoncam:  # get diagnostics on photontags
                SmartDashboard.putBoolean('photoncam_targets_exist', has_photontag)
                if has_photontag:
                    try:
                        ambiguity = self.photoncam_arducam_a.getLatestResult().getBestTarget().getPoseAmbiguity()
                    except AttributeError as e:
                        ambiguity = 998
                    SmartDashboard.putNumber('photoncam_ambiguity', ambiguity)
                else:
                    SmartDashboard.putNumber('photoncam_ambiguity', 997)

        if self.vision_odometry and self.use_quest and self.quest_has_synched and self.counter % 5 == 0:
            # print('quest pose synced')
            quest_accepted = SmartDashboard.getBoolean("QUEST_POSE_ACCEPTED", False)
            quest_pose = self.questnav.get_pose().transformBy(self.quest_to_robot)
            delta_pos = Translation2d.distance(self.get_pose().translation(), quest_pose.translation())
            if delta_pos < 5 and quest_accepted:  # if the quest is way off, we don't want to update from it
                self.pose_estimator.addVisionMeasurement(quest_pose, Timer.getFPGATimestamp(),
                                                         constants.DrivetrainConstants.k_pose_stdevs_disabled)

        if self.vision_odometry and self.use_CJH_apriltags:  # loop through all of our subscribers above
            for count_subscriber, pose_subscriber in zip(self.count_subscribers, self.pose_subscribers):
                # print(f"count subscriber says it has {count_subscriber.get()} tags")
                if count_subscriber.get() > 0:  # use this camera's tag
                    # update pose from apriltags
                    tag_data = pose_subscriber.get()  # 8 items - timestamp, id, tx ty tx rx ry rz
                    id = tag_data[0]
                    tx, ty, tz = tag_data[2], tag_data[3], tag_data[4]
                    rx, ry, rz = tag_data[5], tag_data[6], tag_data[7]
                    tag_pose = Pose3d(Translation3d(tx, ty, tz), Rotation3d(rx, ry, rz)).toPose2d()

                    use_tag = constants.USE_CJH_TAGS  # can disable this in constants
                    # do not allow large jumps when enabled
                    delta_pos = Translation2d.distance(self.get_pose().translation(), tag_pose.translation())
                    # 20251018 commented out the 1m sanity check in case the questnav dies - this way we can get back
                    # use_tag = False if (delta_pos > 1 and DriverStation.isEnabled()) else use_tag  # no big movements in odometry from tags
                    use_tag = False if self._gyro.getRate() > 90 else use_tag  # no more than n degrees per second turning if using a tag
                    # use_tag = False if id not in self.desired_tags else use_tag

                    # TODO - figure out ambiguity (maybe pass to NT from the pi)
                    # do i have a fatal lag issue?  am i better without the time estimate?
                    # based on https://www.chiefdelphi.com/t/swerve-drive-pose-estimator-and-add-vision-measurement-using-limelight-is-very-jittery/453306/13
                    # I gave a fairly high x and y, and a very high theta
                    if use_tag:
                        # print(f'adding vision measurement at {wpilib.getTime()}')
                        sdevs = constants.DrivetrainConstants.k_pose_stdevs_large if DriverStation.isEnabled() else constants.DrivetrainConstants.k_pose_stdevs_disabled
                        self.pose_estimator.addVisionMeasurement(tag_pose, tag_data[0], sdevs)

        # Update the odometry in the periodic block -
        if self.vision_odometry and RobotBase.isReal():
            # self.odometry.update(Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions(),)
            self.pose_estimator.updateWithTime(Timer.getFPGATimestamp(),
                                               Rotation2d.fromDegrees(self._gyro.angle),
                                               self.get_module_positions())

        # in sim, we update from physics.py
        # TODO: if we want to be cool and have spare time, we could use SparkBaseSim with FlywheelSim to do
        # actual physics simulation on the swerve modules instead of assuming perfect behavior

        if self.vision_odometry and self.counter % 10 == 0:
            pose = self.get_pose()
            if True:  # RobotBase.isReal():  # update the NT with odometry for the dashboard - sim will do its own
                SmartDashboard.putNumberArray('drive_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])
                SmartDashboard.putNumber('drive_x', pose.X())
                SmartDashboard.putNumber('drive_y', pose.Y())
                SmartDashboard.putNumber('drive_theta', pose.rotation().degrees())

            # monitor power as well
            if True:  # RobotBase.isReal():
                # there's some kind of voltage simulation but idk if this covers it
                voltage = self.pdh.getVoltage()
                total_current = self.pdh.getTotalCurrent()
            else:
                # make up a current based on how fast we're going
                total_current = 2 + 10 * sum(
                    [math.fabs(module.drivingEncoder.getVelocity()) for module in self.swerve_modules])
                voltage = 12.5 - 0.02 * total_current

            SmartDashboard.putNumber('_pdh_voltage', voltage)
            SmartDashboard.putNumber('_pdh_current', total_current)

            if constants.SWERVE_DEBUG_MESSAGES:  # this is just a bit much unless debugging the swerve
                angles = [m.turningEncoder.getPosition() for m in self.swerve_modules]
                absolutes = [m.get_turn_encoder() for m in self.swerve_modules]
                for idx, absolute in enumerate(absolutes):
                    SmartDashboard.putNumber(f"absolute {idx}", absolute)

                SmartDashboard.putNumberArray(f'_angles', angles)
                # SmartDashboard.putNumberArray(f'_analog_radians', absolutes)

        # Import pose from QuestNav.
        if self.vision_odometry:
            self.quest_periodic()

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller

        # self._gyro.sim_init(physics_controller)

        # # kinematics chassis speeds wants them in same order as in original definition - unfortunate ordering
        # spark_drives = ['lf_drive', 'rf_drive', 'lb_drive', 'rb_drive']
        # spark_drive_ids = [21, 25, 23, 27]  # keep in this order - based on our kinematics definition
        # self.spark_turns = ['lf_turn', 'rf_turn', 'lb_turn', 'rb_turn']
        # spark_turn_ids = [20, 24, 22, 26]  # keep in this order
        #
        # # Got rid of last year's elements: 'br_crank', 'bl_crank', 'tr_crank', 'tl_crank', 't_shooter', 'b_shooter'
        # spark_peripherals = ['intake', 'indexer']
        # spark_peripheral_ids = [5, 12]  # Kept  'indexer' id as 12 because it came last before removing the elements
        #
        # # allow ourselves to access the sim device's Position, Velocity, Applied Output, etc
        # spark_names = spark_drives + self.spark_turns + spark_peripherals
        # spark_ids = spark_drive_ids + spark_turn_ids + spark_peripheral_ids
        #
        # # create a dictionary so we can refer to the sparks by name and get their relevant parameters
        # self.spark_dict = {}
        #
        # for idx, (spark_name, can_id) in enumerate(zip(spark_names, spark_ids)):
        #     spark = simulation.SimDeviceSim(f'SPARK MAX [{can_id}]')
        #
        #     self.spark_dict[spark_name] = {
        #         'controller': spark,
        #         'position': spark.getDouble('Position'),
        #         'velocity': spark.getDouble('Velocity'),
        #         'output': spark.getDouble('Applied Output')
        #     }

        # set up the initial location of the robot on the field
        self._alliance_change(self._container.is_red_alliance,
                              self._container.alliance_location)

    def simulationPeriodic(self, **kwargs) -> Optional[float]:
        """
        This method is called periodically by the CommandScheduler (after the periodic
        function. It is useful for updating subsystem-specific state that needs to be
        maintained for simulations, such as for updating simulation classes and setting
        simulated sensor readings.

        Unlike the physics 'update_sim', it is not called with the current time (now)
        or the amount of time since 'update_sim' was called (tm_diff).  It is called
        just after the 'periodic' call and before the 'update_sim' is called.

        To unify the two uses, our call signature above has a kwargs parameter so we
        know when we are being called. Typically, we only need to support one method
        but for future simulation purposes, if called with keywords, return the amperage
        used in this interval
        """
        # For the swerve drive, we only support the 'update_sim' form of call
        if not kwargs:
            return None

        now, tm_diff = kwargs["now"], kwargs["tm_diff"]
        amperes_used = 0.0  # TODO: Support in future

        log_it = self._robot.counter % 20 == 0

        if log_it:
            logger.debug("Update swerve:===========================================")
            logger.debug(f"Update swerve: Entry. tm_diff: {tm_diff:.4f}")

        dash_values = ['lf_target_vel_angle', 'rf_target_vel_angle', 'lb_target_vel_angle', 'rb_target_vel_angle']
        target_angles = [SmartDashboard.getNumberArray(dash_value, [0, 0])[1] for dash_value in dash_values]
        for spark_turn, target_angle in zip(self.spark_turns, target_angles):
            self.spark_dict[spark_turn]['position'].set(target_angle)  # this works to update the simulated spark

        if self._robot.counter % 10 == 0 and self._robot.isEnabled():
            SmartDashboard.putNumberArray('target_angles', target_angles)

        # using our own kinematics to update the chassis speeds
        module_states = self.get_desired_swerve_module_states()
        speeds = self.kinematics.toChassisSpeeds(module_states)

        if log_it:
            logger.debug(f"Update swerve before drive command: module states: {module_states}, speeds: {speeds}")

        # update the sim's robot. Returned value is same as what is returned from self._physics_controller.get_pose()
        pose = self._physics_controller.drive(speeds, tm_diff)

        # Limit it to the field size (manually)
        robot_x_offset = kwargs.get("robot_x_offset", .4)
        robot_y_offset = kwargs.get("robot_y_offset", .4)
        x = min(FIELD_X_SIZE - robot_x_offset, max(robot_x_offset, pose.x))
        y = min(FIELD_Y_SIZE - robot_y_offset, max(robot_y_offset, pose.y))

        new_pose = Pose2d(x, y, pose.rotation())

        self.resetSimPose(new_pose, [SwerveModulePosition()] * 4,
                          self._physics_controller.get_pose().rotation())
        previous = self.gyro.sim_yaw
        omega = speeds.omega
        gyro_degrees = math.degrees(speeds.omega * tm_diff)
        new = previous - math.degrees(speeds.omega * tm_diff)

        if log_it:
            logger.debug(f"Update swerve: previous: {previous}, new: {new}, omega: {omega}, degrees: {gyro_degrees}")

        self.gyro.sim_yaw = pose.rotation().degrees()

        return amperes_used

    def getHeading(self) -> Rotation2d:
        return self.get_pose().rotation()

    @autolog_output(key="Odometry/Robot")
    def get_pose(self) -> Pose2d:
        """Returns the currently-estimated pose of the robot.

        :returns: The pose.
        """
        estimated = self.pose_estimator.getEstimatedPosition()  # TODO: Support pose_estimator fully
        odometry = self.odometry.getPose()
        return odometry  # TODO: Move to estimated SOON !!!

    def set_pose(self, pose: Pose2d) -> None:
        # TODO: Lots of places could benefit from our getPositions call. Such as the one below
        positions = (self.frontLeft.getPosition(), self.frontRight.getPosition(),
                     self.rearLeft.getPosition(), self.rearRight.getPosition())

        self.odometry.resetPosition(pose.rotation() - self.odometryHeadingOffset, positions, pose)

        self.pose_estimator.resetPosition(self.rawGyroRotation,
                                          positions,
                                          pose)

    if USE_PYKIT:
        # TODO: FOLLOWING need to support swerve drive.
        @autolog_output(key="Drive/LeftPosition")
        def getLeftPosition(self) -> float:
            return self.inputs.leftPositionRad * driveconstants.kWheelRadius

        @autolog_output(key="Drive/RightPosition")
        def getRightPosition(self) -> float:
            return self.inputs.rightPositionRad * driveconstants.kWheelRadius

        @autolog_output(key="Drive/LeftVelocity")
        def getLeftVelocity(self) -> float:
            return self.inputs.leftVelocityRadPerSec * driveconstants.kWheelRadius

        @autolog_output(key="Drive/RightVelocity")
        def getRightVelocity(self) -> float:
            return self.inputs.rightVelocityRadPerSec * driveconstants.kWheelRadius

    def resetSimPose(self, pose, wheel_positions, rotation):
        """
        Currently just used in simulation
        """
        if RobotBase.isSimulation():
            if self.vision_odometry:
                self.pose_estimator.resetPosition(gyroAngle=rotation, wheelPositions=wheel_positions, pose=pose)
            else:
                self.pose_estimator.resetPosition(gyroAngle=rotation, wheelPositions=wheel_positions, pose=pose)
                self.odometry.resetPosition(gyroAngle=rotation, wheelPositions=wheel_positions, pose=pose)

    def resetOdometry(self, pose: Pose2d, resetGyro=True) -> None:
        """Resets the odometry to the specified pose.

        :param pose: The pose to which to set the odometry.

        """
        if self.vision_odometry:
            self.pose_estimator.resetPosition(Rotation2d.fromDegrees(self._gyro.angle),
                                              self.get_module_positions(),
                                              pose)
            return

        if resetGyro and False:
            self._gyro.reset()
            # self._gyro.setAngleAdjustment(0)   # TODO: Look into this?
            self._lastGyroAngleAdjustment = 0
            self._lastGyroAngleTime = 0
            self._lastGyroAngle = 0

        if self.odometry:
            positions = (self.frontLeft.getPosition(), self.frontRight.getPosition(),
                         self.rearLeft.getPosition(), self.rearRight.getPosition())

            heading = self._gyro.heading
            self.odometry.resetPosition(heading, positions, pose)

            self.odometryHeadingOffset = self.odometry.getPose().rotation() - heading

    def adjustOdometry(self, dTrans: Translation2d, dRot: Rotation2d):
        pose = self.get_pose()
        newPose = Pose2d(pose.translation() + dTrans, pose.rotation() + dRot)

        positions = (self.frontLeft.getPosition(), self.frontRight.getPosition(),
                     self.rearLeft.getPosition(), self.rearRight.getPosition())

        self.odometry.resetPosition(pose.rotation() - self.odometryHeadingOffset, positions, newPose)
        self.odometryHeadingOffset += dRot

    def stop(self):
        # if USE_PYKIT:
        #     self.runOpenLoop(0, 0)
        # else:
        #     self.arcadeDrive(0, 0)
        pass

    def arcadeDrive(self, x_speed: float, rot: float, assumeManualInput: bool = False) -> None:
        self.drive(x_speed, 0, rot, square=assumeManualInput)

    def rotate(self, rotSpeed) -> None:
        """
        Rotate the robot in place, without moving laterally (for example, for aiming)
        :param rotSpeed: rotation speed
        """
        self.arcadeDrive(0, rotSpeed)

    def drive(self, x_speed: float, y_speed: float, rotation: float, fieldRelative: Optional[bool] = False,
              rateLimit: Optional[bool] = False, square: Optional[bool] = False) -> None:

        """Method to drive the robot using joystick info.

        :param x_speed:       Speed of the robot in the x direction (forward).
        :param y_speed:       Speed of the robot in the y direction (sideways).
        :param rotation:      Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        :param square:        Whether to square the inputs (useful for manual control)
        """
        if square:
            rotation = rotation * abs(rotation)
            norm = math.sqrt(x_speed * x_speed + y_speed * y_speed)
            x_speed = x_speed * norm
            y_speed = y_speed * norm

        xSpeedCommanded = x_speed
        ySpeedCommanded = y_speed

        if rateLimit:
            # TODO: Currently, this is not being used.   Probably want before too long
            # Convert XY to polar for rate limiting
            inputTranslationDir = math.atan2(y_speed, x_speed)
            inputTranslationMag = math.hypot(x_speed, y_speed)

            # Calculate the direction slew rate based on an estimate of the lateral acceleration
            if self.currentTranslationMag != 0.0:
                directionSlewRate = abs(DriveConstants.DIRECTION_SLEW_RATE / self.currentTranslationMag)
            else:
                directionSlewRate = 500.0
                # some high number that means the slew rate is effectively instantaneous

            currentTime = Timer.getFPGATimestamp()
            elapsedTime = currentTime - self.prevTime
            angleDif = swerveutils.angleDifference(inputTranslationDir, self.currentTranslationDir)

            if angleDif < 0.45 * math.pi:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(inputTranslationMag)

            elif angleDif > 0.85 * math.pi:
                # some small number to avoid floating-point errors with equality checking
                # keep currentTranslationDir unchanged
                if self.currentTranslationMag > 1e-4:
                    self.currentTranslationMag = self.magLimiter.calculate(0.0)
                else:
                    self.currentTranslationDir = swerveutils.wrapAngle(self.currentTranslationDir + math.pi)
                    self.currentTranslationMag = self.magLimiter.calculate(inputTranslationMag)

            else:
                self.currentTranslationDir = swerveutils.stepTowardsCircular(
                    self.currentTranslationDir,
                    inputTranslationDir,
                    directionSlewRate * elapsedTime,
                )
                self.currentTranslationMag = self.magLimiter.calculate(0.0)

            self.prevTime = currentTime

            xSpeedCommanded = self.currentTranslationMag * math.cos(self.currentTranslationDir)
            ySpeedCommanded = self.currentTranslationMag * math.sin(self.currentTranslationDir)
            self.currentRotation = self.rotLimiter.calculate(rotation)

        else:
            self.currentRotation = rotation

        # We scale our speed down during development
        scaler = self._container.chosenLimiter.getSelected()
        if not isinstance(scaler, (int, float)) or scaler < 0.0 or scaler > 1.0:
            logger.info(f"Invalid drive rate limiter: '{scaler}")
            scaler = 0.1

        # Convert the commanded speeds into the correct units for the drivetrain
        self.xSpeedDelivered = xSpeedCommanded * DriveConstants.MAX_SPEED_METERS_PER_SECOND * scaler
        self.ySpeedDelivered = ySpeedCommanded * DriveConstants.MAX_SPEED_METERS_PER_SECOND * scaler
        self.rotDelivered = self.currentRotation * DriveConstants.MAX_ANGULAR_SPEED * scaler

        if fieldRelative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeedDelivered, self.ySpeedDelivered,
                                                           self.rotDelivered, self._gyro.heading)
        else:
            speeds = ChassisSpeeds(self.xSpeedDelivered, self.ySpeedDelivered, self.rotDelivered)

        swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds)

        max_speed = DriveConstants.MAX_SPEED_METERS_PER_SECOND

        if self.maxSpeedScaleFactor is not None:
            max_speed = max_speed * self.maxSpeedScaleFactor()

        fl, fr, rl, rr = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, max_speed)
        self.frontLeft.setDesiredState(fl)
        self.frontRight.setDesiredState(fr)
        self.rearLeft.setDesiredState(rl)
        self.rearRight.setDesiredState(rr)

    def set_x_formation(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        self.frontLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))
        self.frontRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearLeft.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(-45)))
        self.rearRight.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(45)))

    def set_straight(self) -> None:
        """Sets the wheels straight so we can push the robot."""
        angles = [0, 0, 0, 0]
        for angle, swerve_module in zip(angles, self.swerve_modules):
            swerve_module.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(angle)))

    def setModuleStates(self, desiredStates: SwerveModuleStats) -> None:
        """Sets the swerve ModuleStates.

        :param desiredStates: The desired SwerveModule states.
        """
        maxSpeed = DriveConstants.MAX_SPEED_METERS_PER_SECOND

        if self.maxSpeedScaleFactor is not None:
            maxSpeed = maxSpeed * self.maxSpeedScaleFactor()

        desiredStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, maxSpeed)

        for idx, module in enumerate(self.swerve_modules):
            module.setDesiredState(desiredStates[idx])

    def set_desired_april_tags(self, desired_tags: List[int]) -> None:
        self._desired_april_tags = desired_tags

    def resetEncoders(self) -> None:
        """
        Resets the drive encoders to currently read a position of 0
        """
        [m.resetEncoders() for m in self.swerve_modules]

    def resetGyroToInitial(self) -> None:
        self.gyro.reset()

    def setMotorBrake(self, brake: bool) -> None:
        # TODO: Need to actually set the IdleMode to 'brake' since this
        #       would be useful on an incline as well
        # if brake:
        #     for motor in (self.rearLeft, self.rearRight, self.frontLeft, self.frontRight):
        #         motor.stop()
        #     self.set_x_formation()
        pass

    def get_module_positions(self) -> list: # Generator[SwerveModulePosition, Any, None]:
        return []
        # return (m.getPosition() for m in self.swerve_modules)

    def get_module_states(self) -> list:  # List[SwerveModuleState]:
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [] # [m.getState() for m in self.swerve_modules]

    def get_angle(self) -> degrees:
        return self.get_pose().rotation().degrees()

    def get_desired_swerve_module_states(self) -> List[SwerveModuleState]:
        """
        what it says on the wrapper; it's for physics.py because I don't like relying on an NT entry
        to communicate between them (it's less clear what the NT entry is there for, I think) LHACK 1/12/25
        """
        return [module.getDesiredState() for module in self.swerve_modules]

    ##########################################################
    # TODO: All the following are related to team 2429 and pathplanner. These have not been tested and
    #       we may need to refactor the 'drive()' method above

    #  -------------  THINGS PATHPLANNER NEEDS  - added for pathplanner 20230218 CJH
    def get_relative_speeds(self) -> ChassisSpeeds:
        # added for pathplanner 20230218 CJH
        return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(self.get_module_states())

    def drive_robot_relative(self, chassis_speeds: ChassisSpeeds, feedforwards):
        """
        feedforwards isn't used at all so pass it whatever
        """
        # required for the pathplanner lib's path following based on chassis speeds
        # idk if we need the feedforwards
        swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(chassis_speeds)
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates,
                                                                          DriveConstants.MAX_TOTAL_SPEED)
        for state, module in zip(swerveModuleStates, self.swerve_modules):
            module.setDesiredState(state)

    # def follow_pathplanner_trajectory_command(self, trajectory:PathPlannerTrajectory, is_first_path:bool):
    #     #from pathplannerlib.path import PathPlannerPath
    #     #from pathplannerlib.commands import FollowPathWithEvents, FollowPathHolonomic
    #     #from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants

    #     # copy of pathplannerlib's method for returning a swervecommand, with an optional odometry reset
    #     # using the first pose of the trajectory
    #     if is_first_path:
    #         reset_cmd = commands2.InstantCommand(self.resetOdometry(trajectory.getInitialTargetHolonomicPose()))
    #     else:
    #         reset_cmd = commands2.InstantCommand()

    #     # useful stuff controller.PPHolonomicDriveController, controller.PIDController, auto.FollowPathHolonomic
    #     swerve_controller_cmd = None

    #     cmd = commands2.SequentialCommandGroup(reset_cmd, swerve_controller_cmd)

    #     return cmd

    #  END PATHPLANNER STUFF
