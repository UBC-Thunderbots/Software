from enum import Enum

import ipaddress


class RobotPlatform(Enum):
    """Enum of the supported robot platforms (compute modules)"""

    NANO = "NANO"
    PI = "PI"


class NetworkConstants:
    """Constants for obtaining the IP addresses of robots"""

    SUBNETS = list(ipaddress.ip_network("192.168.0.0/16").subnets(new_prefix=24))

    NANO_SUBNET: ipaddress.IPv4Network = SUBNETS[0]
    PI_WIFI_5_SUBNET: ipaddress.IPv4Network = SUBNETS[5]
    PI_WIFI_6_SUBNET: ipaddress.IPv4Network = SUBNETS[6]

    HOST_PORTION_BASE = 200

    @staticmethod
    def get_ip_address(robot_id: int, platform: RobotPlatform) -> ipaddress.IPv4Address:
        """Get the IP address of a robot

        :param robot_id: the robot ID
        :param platform: the robot's platform (compute module)
        :return: the IP address of the robot
        """
        host = NetworkConstants.HOST_PORTION_BASE + robot_id
        if platform == RobotPlatform.NANO:
            return NetworkConstants.NANO_SUBNET[host]
        elif platform == RobotPlatform.PI:
            return NetworkConstants.PI_WIFI_5_SUBNET[host]
