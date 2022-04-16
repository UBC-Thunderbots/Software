from software.networking import networking
from software.py_constants import *
from software.estop.estop_reader import ThreadedEstopReader


class RobotCommunication(object):

    """ Communicate with the robots """

    def __init__(self, multicast_channel, interface):
        """Initialize the communication with the robots

        :param multicast_channel: The multicast channel to use

        """
        self.receive_robot_status = networking.RobotStatusProtoListener(
            multicast_channel + "%" + interface,
            ROBOT_STATUS_PORT,
            self.__receive_robot_status,
            True,
        )

        # TODO make this automagical
        # self.estop_reader = ThreadedEstopReader("/dev/ttyACM0", 115200)

    def __receive_robot_status(self, data):
        """Receive the status of the robots

        :param data: The data received

        """
        print(data)
