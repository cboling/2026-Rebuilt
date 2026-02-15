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


def test_module_offsets(control, robot):
    """
    While a swerve drive can drive in any direction, the notion of front/back/left/right
    still exists, and we give offsets to these based off of the center of the robot.  So
    check that they are correct polarity +/-
    """
    pass


def test_no_duplicate_can_bus_ids(control, robot):
    """
    Run through our constants and make sure they are unique and match up
    """
    pass


def test_operator_control(control, robot):
    """
    This test checks to see if a joystick input causes a corresponding
    motor output. Obviously this is a silly example, but you can build
    upon it to make much more comprehensive (and stateful) tests.

    Keep in mind that when you set a value, the robot code does not
    see the value immediately, but only when teleopPeriodic is called,
    which is typically every 20ms
    """

    # run_robot will cause the robot to be initialized and robotInit to be called
    # with control.run_robot():
    #     motorsim = wpilib.simulation.PWMSim(robot.motor.getChannel())
    #     joysim = wpilib.simulation.JoystickSim(robot.lstick)
    #
    #     # Set the joystick value to something
    #     joysim.setY(0.5)
    #
    #     # Enable the robot, check to see if it was set
    #     control.step_timing(seconds=0.1, autonomous=False, enabled=True)
    #     assert 0.5 == pytest.approx(motorsim.getSpeed(), rel=0.01)
    #
    #     # change it, see if it's still set
    #     joysim.setY(0.2)
    #     control.step_timing(seconds=0.1, autonomous=False, enabled=True)
    #     assert 0.2 == pytest.approx(motorsim.getSpeed(), rel=0.01)
