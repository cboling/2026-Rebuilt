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
from typing import Any, Dict, Optional, Tuple, List, Callable

from commands2 import Subsystem
from ntcore import NetworkTable, NetworkTableInstance
from robotpy_apriltag import AprilTagDetector, AprilTagField, AprilTagFieldLayout
from wpilib import Alert, RobotBase, SmartDashboard
from wpimath.geometry import Rotation2d, Transform3d, Pose3d, Pose2d
from wpimath.units import degrees, milliseconds, percent, seconds
from pykit.logger import Logger

import constants
from lib_6107.subsystems.pykit.vision_io import VisionIO,PoseObservation, PoseObservationType
from lib_6107.util.field import Field

logger = logging.getLogger(__name__)

VisionConsumer = Callable[[Pose2d, seconds, tuple[float,float,float] | None], None]

class VisionTargetData:
    def __init__(self, yaw: degrees, pitch: degrees, area: percent,
                 fudicial_id: int, pose_ambiguity: float,
                 best_target_transform: Transform3d,
                 alternate_target_transform: Transform3d):
        # The yaw of the target in degrees (positive left)
        self.yaw: degrees = yaw

        # The pitch of the target in degrees (positive up)
        self.pitch: degrees = pitch

        # The area (how much of the camera feed the bounding box takes up) as a percent (0-100)
        self.area: percent = area

        # AprilTag related data (if any)
        # The ID of the detected fiducial marker
        self.april_tag_id: int = fudicial_id

        # How ambiguous the pose of the target is
        self.pose_ambiguity: float = pose_ambiguity

        # Get the transform that maps camera space (X = forward, Y = left, Z = up)
        # to object/fiducial tag space (X forward, Y left, Z up) with the lowest
        # reprojection error
        self.best_target_transform: Transform3d = best_target_transform

        # Get the transform that maps camera space (X = forward, Y = left, Z = up)
        # to object/fiducial tag space (X forward, Y left, Z up) with the highest
        # reprojection error
        self.alternate_target_transform: Transform3d = alternate_target_transform


class VisionSubsystem(Subsystem, VisionIO):
    """
    Vision Subsystem

    Currently only support a single camera per vision subsystem. For the PhotonVision, we
    'could' eventually support multiple cameras from a single PhotonVision unit.

    TODO: Add multi-camera per VisionSubsystem support. Along these lines, we could just have a single
          vision subsystem, but have it run any combination of cameras...
    """
    def __init__(self, vision_input: VisionConsumer,
                camera_name: str, field: Field,
                transform: Transform3d, drivetrain: 'DriveSubsystem'):
        Subsystem.__init__(self)
        VisionIO().__init__()

        # Load the initial field layout. This can be changed later at
        # the beginning of the Autonomous or Teleop periods
        self._april_tag_field: Optional[AprilTagField] = field.field
        self._field_layout: Optional[AprilTagFieldLayout] = field.layout
        self._name = camera_name
        self._robot = drivetrain.robot
        self._vision_input = vision_input

        self._camera_transform: Transform3d = transform
        self._drivetrain: 'DriveSubsystem' = drivetrain
        self._is_simulation: bool = RobotBase.isSimulation()

        self._inputs = VisionIO.VisionIOInputs()

        # April tag setup
        self._tag_detector = AprilTagDetector()
        self._tag_detector.addFamily("tag16h5")

        # NetworkTable setup
        nt_instance = NetworkTableInstance.getDefault()
        self._network_table: NetworkTable = nt_instance.getTable(camera_name)

        # TODO: Also look at AdvantageKit java 'Vision.java' for additional work
        #       needed here.
        # Initialize disconnected alerts
        self._disconnected_alert = Alert(f"Vision camera {self.name}", Alert.AlertType.kWarning)

        # TODO: Make following a programable value so we can trust some cameras more than others
        self._camera_std_dev_factors: float = 1.0

    @staticmethod
    def create(vision_input: VisionConsumer,
               info: Dict[str, Any], field: Field,
               drivetrain: 'DriveSubsystem') -> Tuple[Optional['VisionSubsystem'], Optional['Subsystem']]:

        camera_type = info.get("Type", constants.CAMERA_TYPE_NONE)
        localizer = info.get("Localizer")
        transform: Transform3d = info.get("Pose")
        camera_name = info.get("Name", camera_type)
        heading = info.get("Heading", Rotation2d.fromDegrees(0))

        camera_subsystem: Optional['VisionSubsystem'] = None
        localizer_subsystem: Optional['Subsystem'] = None

        match constants.ROBOT_MODE:
            case constants.RobotModes.REAL:
                pass  # TODO: In the 'match' below, use these values to create a real or simulated camera

            case constants.RobotModes.SIMULATION:
                pass  # TODO: In the 'match' below, use these values to create a real or simulated camera

            case constants.RobotModes.REPLAY:
                pass  # TODO: In the 'match' below, use this to create a simulated replay camera

        # TODO: For items above, look at the RobotContainer.java from the vision template for AdvantageKit

        match camera_type:
            case constants.CAMERA_TYPE_LIMELIGHT:
                # TODO: For limelight, allow multiple cameras to be specified
                from lib_6107.subsystems.vision.limelightvision import LimelightVisionSubsystem
                camera_subsystem = LimelightVisionSubsystem(vision_input, camera_name, field, transform, drivetrain)
                # self.localizer = LimelightLocalizer(self, self.robot_drive)
                # self.localizer.addCamera(self.camera,
                #                          cameraPoseOnRobot=pose["Pose"],
                #                          cameraHeadingOnRobot=pose["Heading"])

            case constants.CAMERA_TYPE_PHOTONVISION:
                try:
                    from lib_6107.subsystems.vision.photonvision import PhotonVisionSubsystem
                    camera_subsystem = PhotonVisionSubsystem(vision_input, camera_name, field, transform, drivetrain)

                    # if localizer:
                    #     pass # TODO: Need to support
                    #     localizer_subsystem = PhotonLocalizer(self, self.robot_drive, "2025-reefscape.json")
                except ImportError as e:
                    logger.error(f"PhotonVisionSubsystem not found, camera {camera_name}: {e}")

        return camera_subsystem, localizer_subsystem

    @property
    def name(self) -> str:
        return self._name

    @property
    def inputs(self) -> VisionIO.VisionIOInputs:
        return self._inputs

    @property
    def drivetrain(self) -> 'DriveSubsystem':
        return self._drivetrain

    @property
    def april_tag_field(self) -> AprilTagField:
        return self._april_tag_field

    @property
    def pipeline(self) -> int:
        raise NotImplementedError("TODO: Implement in subclass")

    @pipeline.setter
    def pipeline(self, index: int) -> None:
        raise NotImplementedError("TODO: Implement in subclass")

    @property
    def latency(self) -> Optional[milliseconds]:
        return None

    @property
    def timestamp(self) -> Optional[seconds]:
        """
        Returns the estimated time the frame was taken, in the Received system's time base
        """
        raise NotImplementedError("TODO: Implement in subclass")

    @property
    def best_target(self) -> Optional[VisionTargetData]:
        """
        Returns the best target in this pipeline result. If there are no targets, this method will
        return null. The best target is determined by the target sort mode in the PhotonVision UI.
        """
        raise NotImplementedError("TODO: Implement in subclass")

    @property
    def valid(self) -> bool:
        raise NotImplementedError("TODO: Implement in subclass")

    @property
    def area(self) -> percent:
        """
        Target Area (0..100] percent of image
        """
        raise NotImplementedError("TODO: Implement in subclass")

    @property
    def x_offset(self) -> degrees:
        """
        Horizontal Offset from Crosshair to Target
        """
        raise NotImplementedError("TODO: Implement in subclass")

    @property
    def y_offset(self) -> degrees:
        """
        Vertical Offset from Crosshair to Target
        """
        raise NotImplementedError("TODO: Implement in subclass")

    def get_latest_results(self) -> Optional[Any]:
        raise NotImplementedError("Implement in subclass")

    def periodic(self):
        # Update inputs
        inputs: VisionIO.VisionIOInputs = self.inputs

        self.updateInputs(inputs)
        # TODO: Once one subsystem supports multiple cameras, need to track 'all' poses
        #       instead of just one camera's worth.  See AdvantageKit vision.java example
        # # Initialize logging values
        # allTagPoses: List[Pose3d] = []
        # allRobotPoses: List[Pose3d] = []
        # allRobotPosesAccepted: List[Pose3d] = []
        # allRobotPosesRejected: List[Pose3d] = []

        # Update disconnected alert
        self._disconnected_alert.set(not inputs.connected)

        # Initialize logging values           TODO: Get these working
        tag_poses: List[Pose3d] = []
        robot_poses: List[Pose3d] = []
        robot_poses_accepted: List[Pose3d] = []
        robot_poses_rejected: List[Pose3d] = []

        # Add tag poses
        tag_ids = inputs.tag_ids or []

        for tag_id in tag_ids:
            tag_pose: Pose3d | None = self._field_layout.getTagPose(tag_id)
            if tag_pose:
                tag_poses.append(tag_pose)

        # Loop over pose observations
        observations: List[PoseObservation] = inputs.pose_observations or []
        for observation in observations:
            # Check whether to reject pose
            #   - Must have at least one tag
            #   - Cannot be high ambiguity
            #   - Must have realistic Z coordinate
            #   - Must be within the field boundaries
            x, y, z = observation.pose.X(), observation.pose.Y(), observation.pose.Z()

            reject_pose = observation.tag_count == 0 or \
                (observation.tag_count == 1 and observation.ambiguity > constants.MAX_VISION_AMBIGUITY) or \
                abs(z) > constants.MAX_VISION_Z_ERROR or \
                x < 0.0 or \
                y < 0.0 or \
                x > self._field_layout.getFieldLength() or \
                y > self._field_layout.getFieldWidth()

            # Add pose to log
            robot_poses.append(observation.pose)

            if reject_pose:
                robot_poses_rejected.append(observation.pose)
            else:
                robot_poses_accepted.append(observation.pose)

            # Skip if rejected
            if reject_pose:
                continue

            if self._vision_input:
                # Calculate standard deviations
                # TODO: The CTRE drivetrain also supports vision standard deviations.  See how this and our
                #       new vision constants interact with those values
                std_dev_factor: float = math.pow(observation.avg_tag_distance, 2.0) / observation.tag_count
                linear_std_dev: float = constants.LINEAR_STD_DEV_BASELINE * std_dev_factor
                angular_std_dev: float = constants.ANGULAR_STD_DEV_BASELINE * std_dev_factor

                if observation.observation_type == PoseObservationType.MEGATAG_2:
                    linear_std_dev *= constants.LINEAR_STD_DEV_MEGATAG2_FACTOR
                    angular_std_dev *= constants.ANGULAR_STD_DEV_MEGATAG2_FACTOR

                # TODO: Support sending to drivetrain vision measurements here instead of elsewhere

                linear_std_dev *= self._camera_std_dev_factors
                angular_std_dev *= self._camera_std_dev_factors

                # Send vision observation       # TODO: Validate call tuples belos
                self._vision_input(observation.pose.toPose2d(), observation.timestamp,
                                   (linear_std_dev, linear_std_dev, angular_std_dev))

        # Log camera metadata
        Logger.recordOutput(f"Vision/Camera/{self.name}/TagPoses", tag_poses)
        Logger.recordOutput(f"Vision/Camera/{self.name}/RobotPoses", robot_poses)
        Logger.recordOutput(f"Vision/Camera/{self.name}/RobotPosesAccepted", robot_poses_accepted)
        Logger.recordOutput(f"Vision/Camera/{self.name}/RobotPosesRejected", robot_poses_rejected)

        # TODO: Do following if we ever support multi-camera per subsystem,
        #   allTagPoses.extend(tagPoses);
        #   allRobotPoses.extend(robotPoses);
        #   allRobotPosesAccepted.extend(robotPosesAccepted);
        #   allRobotPosesRejected.extend(robotPosesRejected);
        #
        # Log summary data
        # Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        # Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        # Logger.recordOutput(
        #     "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        # Logger.recordOutput(
        #     "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));

    def simulationPeriodic(self):
        pass  # For now  TODO: Can any of this be common

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        # SmartDashboard.putData("Field", self.field)
        SmartDashboard.putString('Camera/name', self._name)
        # SmartDashboard.putString('Camera/type', "Limelight")

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        pass
        # SmartDashboard.putString('Camera/heartbeat', "Alive" if self.heartbeating else "Dead")
        # SmartDashboard.putNumber('Camera/last-heartbeat', self.lastHeartbeatTime)
