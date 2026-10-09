
from typing import TypedDict



class MCUSchema(TypedDict):
    imu: str
    gnss_fix: str 
    altimeter: str 
    yaw: str 
    odom: str 
    rob_status: str