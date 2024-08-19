from rplidar import RPLidar as rpl, RPLidarException
from serial.tools.list_ports import comports
import numpy as np
from logging import getLogger
from threading import Thread
import time
from _thread import interrupt_main
logger = getLogger(__name__)

def find_port_by_vid_pid(vid, pid):
    ports = list(comports())

    for port in ports:
        if port.vid == vid and port.pid == pid:
            return port.device
    return None


class RP_A1:
    VID = 0x10c4
    PID = 0xea60

    def __init__(self, com="/dev/ttyUSB0", baudrate=115200, timeout=3, rotation=0, scan_type="normal", threaded=True):
        self.exit_called = False
        self.t = threaded
        port = find_port_by_vid_pid(self.VID, self.PID)
        self.lidar = rpl(port, baudrate, timeout)
        try:
            logger.info(f"{self.lidar.get_info(), self.lidar.get_health()}")
        except RPLidarException:
            self.lidar = rpl(port, baudrate, timeout)
            logger.info(f"{self.lidar.get_info(), self.lidar.get_health()}")
        self.lidar.clean_input()
        self.scanner = self.lidar.iter_scans(scan_type, False, 5)

        next(self.scanner)
        self.rotation = rotation % 360
        if threaded:
            self.latest = [[0], [0]]
            self.scans = Thread(target=self.threaded_read, daemon=True)
            self.scans.start()
        else:
            self.scans = None
        # last_scan = self.read()  # Return this for when we must clear the buffer if we don't read fast enough.

    def threaded_read(self):
        try:
            while self.t:
                try:
                    self.latest = self.read()
                except OSError:
                    pass
                except StopIteration:
                    time.sleep(1/30)
                    self.scanner = self.lidar.iter_scans('normal', False, 100)

        except RPLidarException as e:
            logger.error(f"[RPLidar]: {e} | {e.args}")
            interrupt_main()

    def read(self, rotate=False):  # It's totally possible to move this to a thread.
        ''' THIS IS NOT MEANT FOR USE IN MAIN. THIS IS ONLY FOR INTERNAL CLASS USE. DO NOT TOUCH!!!'''
        items = next(self.scanner)
        angles, distances = list(zip(*items))[1:]
        if rotate:
            distances, angles = list(zip(*self.rotate_lidar_readings(zip(distances, angles))))
        return [list(distances), list(angles)]

    def exit(self):
        if self.exit_called:
            return
        if self.t:
            self.t = False
            self.scans.join()
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
        self.exit_called = True

    def readCartesian(self):
        distance, angle = np.array(self.getScan())
        angle = np.deg2rad(360-angle)  # done to un-mirror the lidar, kinda.
        return np.array([distance*np.cos(angle), distance*np.sin(angle)]).T

    def getScan(self):
        return self.latest if self.t else self.read()

    def rotate_lidar_readings(self, readings):
        """
        Rotate lidar angle readings by the specified angle while keeping distances intact.

        Args:
            readings (list): List of tuples containing distances and angles.

        Returns:
            list: Rotated lidar readings with preserved distances.
        """
        if self.rotation % 360 == 0:
            return readings
        else:
            return [(readings[i][0], (readings[i][1] + self.rotation + 360) % 360) for i in range(len(readings))]
