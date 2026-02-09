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

from enum import Enum
from typing import List

try:
    from phoenix6 import StatusCode, StatusSignal

except ImportError:
    class StatusCode(Enum):
        OK = 0


    class StatusSignal:
        @staticmethod
        def refesh_all() -> StatusCode:
            return StatusCode.OK


class Phoenix6Signals:
    """
    Optimizer for CTRE devices.

    """
    _signals: List[StatusSignal] = []

    @classmethod
    def register_signal(cls, signal: StatusSignal) -> None:
        cls._signals.append(signal)

    @classmethod
    def register_signals(cls, *signals: StatusSignal) -> None:
        cls._signals.extend(signals)

    @classmethod
    def refresh(cls) -> StatusCode:
        """
        In robot.robotPeriodic, call this to request all status to be updated at once. Documentation
        says it is faster
        """
        return StatusSignal.refresh_all(*cls._signals)
