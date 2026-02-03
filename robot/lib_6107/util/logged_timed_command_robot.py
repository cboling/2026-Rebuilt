#!/usr/bin/env python3
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

import hal
from commands2 import Subsystem
from commands2 import TimedCommandRobot
# pykit & AdvantageScope support
from pykit.logger import Logger
from wpilib import DSControlWord, RobotController
from wpilib import Watchdog


# from util.telemetry import Telemetry


class LoggerSubsystem(Subsystem):

    def __init__(self, period: Optional[float] = None):
        super().__init__()

        self.use_timing = True
        self.notifier = hal.initializeNotifier()[0]
        self.watchdog = Watchdog(period, self.print_overrun_message)
        self.word = DSControlWord()
        self.user_code_start = 0
        self.periodic_before_start = 0

    def print_overrun_message(self):
        """Prints a message when the main loop overruns."""
        print("Loop overrun detected!")

    def periodic(self) -> None:
        pass

    def startCompetition(self) -> None:
        """
        The main loop of the robot.
        Handles timing, logging, and calling the periodic functions.
        This method replaces the standard `IterativeRobotBase.startCompetition`
        to inject logging and precise timing control.
        """
        init_end = RobotController.getFPGATime()
        Logger.periodicAfterUser(init_end, 0)

        hal.observeUserProgramStarting()

        Logger.startReciever()

    def endCompetition(self) -> None:
        """
        Called at the end of the competition to clean up resources.
        """
        hal.stopNotifier(self.notifier)
        hal.cleanNotifier(self.notifier)

    def robotPeriodic(self):
        # Run logger pre-user code (load inputs from log or sensors).  This is called
        # from the base class 'startCompetition' loop at the start of cycle.

        self.periodic_before_start = RobotController.getFPGATime()
        Logger.periodicBeforeUser()

        # Execute user periodic code and measure timing
        self.user_code_start = RobotController.getFPGATime()

    def endOfLoop(self):
        """
        Loop function from the Iterative Robot base.  Call after remaining user code to
        compute the differences
        """
        user_code_end = RobotController.getFPGATime()

        # Run logger post-user code (save outputs to log)
        Logger.periodicAfterUser(user_code_end - self.user_code_start,
                                 self.user_code_start -
                                 self.periodic_before_start)


class LoggedTimedCommandRobot(TimedCommandRobot):
    """
    Provides a wpilib TimedCommandRobot with pykit logging capabilities.

    This class can be used as your MyRobot base if you need a Timed robot with
    both Commandsv2 and pykit (AdvantageScope) support.

    Since pykit's LoggedRobot does not support specifying the default period in
    the initializer, we will not support that either at this time.
    """
    default_period = 0.02  # seconds

    def __init__(self):
        """
        Constructor for the LoggedTimedCommandRobot.
        Initializes the robot, sets up the logger, and creates I/O objects.
        """
        super().__init__(period=self.default_period)
        self._logger_subsystem = LoggerSubsystem(self.getPeriod())

    def endCompetition(self) -> None:
        """
        Called at the end of the competition to clean up resources.
        """
        self._logger_subsystem.endCompetition()
        super().endCompetition()

    def startCompetition(self) -> None:
        """
        The main loop of the robot.
        Handles timing, logging, and calling the periodic functions.
        This method replaces the standard `IterativeRobotBase.startCompetition`
        to inject logging and precise timing control.
        """
        self._logger_subsystem.startCompetition()

        super().startCompetition()
        pass

    def robotPeriodic(self):
        # Run logger pre-user code (load inputs from log or sensors).  This is called
        # from the base class 'startCompetition' loop at the start of cycle.
        self._logger_subsystem.robotPeriodic()

        super().robotPeriodic()

    def _loopFunc(self):
        """
        Loop function from the Iterative Robot base.  Call all remaining user code and then
        compute the differences
        """
        pass

        super()._loopFunc()

        self._logger_subsystem.endOfLoop()
