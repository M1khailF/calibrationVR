import motorcortex
import triad_openvr
import time
import numpy as np
import json
from math import cos, sin, pi
import threading

class MCX:
    def __init__(self, hostname="192.168.2.100"):
        self.hostname = hostname
        self.connect()
        self.inputCmd = 'root/Control/writingInput'
        self.swithCmd = 'root/Control/autoMotionGenerator/userFrames/switch01/toggle'
        self.currentPoseCmd = 'root/Control/actualToolCoordinates'
        self.ethercatParam = 'root/Ethercat/Robot/EL2024 4K. Dig. Ausgang 24V, 2A/Channel 1'
        self.buttonState = 'root/Control/dummyBool'
        self.robotState = "root/Logic/stateCommand"
        self.x_sensor_control_range = [0.25, 0.35]
        self.y_sensor_control_range = [-0.22, 0.22]
        self.z_sensor_control_range = [0.158, 0.25]

    def connect(self):
        parameter_tree = motorcortex.ParameterTree()
        # Open request and subscribe connection
        try:
            self.req, sub = motorcortex.connect("wss://" + self.hostname + ":5568:5567",
                                                motorcortex.MessageTypes(), parameter_tree,
                                                certificate="1mcx.cert.pem", timeout_ms=1000,
                                                login="admin", password="vectioneer")
        except RuntimeError as err:
            print(err)
            exit()

    def readCurrentPose(self):
        get_param_reply_msg = self.req.getParameter(
            self.currentPoseCmd).get()
        current_pose = get_param_reply_msg.value
        return current_pose

    def setSwith(self, cmd):
        set_param_reply_msg = self.req.setParameter(
            self.swithCmd, cmd).get()
    
    def writePoint(self, point: list):
        print("Writing point {0}".format(point))
        self.req.setParameter(
            self.inputCmd, point + [0, 0, 0]).get()

    def checkAndSetPointConstrain(self, startPose, point):
        pose  = np.array(startPose[:3]) + np.array(point[:3])
        res_point = np.array(point)

        if pose[0] > self.x_sensor_control_range[1]:
            res_point[0] = self.x_sensor_control_range[1] - startPose[0]

        if pose[0] < self.x_sensor_control_range[0]:
            res_point[0] = self.x_sensor_control_range[0] - startPose[0]

        if pose[1] > self.y_sensor_control_range[1]:
            res_point[1] = self.y_sensor_control_range[1] - startPose[1]

        if pose[1] < self.y_sensor_control_range[0]:
            res_point[1] = self.y_sensor_control_range[0] - startPose[1]

        if pose[2] > self.z_sensor_control_range[1]:
            res_point[2] = self.z_sensor_control_range[1] - startPose[2]

        if pose[2] < self.z_sensor_control_range[0]:
            res_point[2] = self.z_sensor_control_range[0] - startPose[2]
        
        # if pose[0] > self.x_sensor_control_range[1]:
        #     pose[0] = self.x_sensor_control_range[1]

        # if pose[1] > self.y_sensor_control_range[1] or pose[1] < self.y_sensor_control_range[0]:
        #     return False

        # if pose[2] > self.z_sensor_control_range[1] or pose[2] < self.z_sensor_control_range[0]:
        #     return False
        
        return res_point.tolist()

    def ethercatParamSet(self,):
        set_param_reply_msg = self.req.setParameter(
            self.ethercatParam, 1).get()
    
    def checkButtonWithHighPrivacy(self):
        def checkAndStopCallback():
            while True:
                set_param_reply_msg = self.req.getParameter(
                self.buttonState).get()
                if set_param_reply_msg.value[0]:
                    print("PUSHED STOP BUTTON")
                    self.req.setParameter(
                        self.robotState, 0).get()
                    self.setSwith(0)
                    raise Exception("PUSHED STOP BUTTON")
                else:
                    time.sleep(0.025)
        
        threading.Thread(target=checkAndStopCallback, daemon=True).start()

def z_rotation(vector, angle):
    z_rot = np.array([[cos(angle), -sin(angle), 0],
                      [sin(angle), cos(angle), 0],
                      [0, 0, 1]])

    res = z_rot@np.array(vector)
    return res.tolist()

def median_filter(new, old):
    alpha = 0.7
    filtered = np.array(new)*(1 - alpha) + np.array(old)*alpha
    return filtered.tolist()

def main():
    mcx = MCX('192.168.2.100')

    mcx.ethercatParamSet()
    mcx.checkButtonWithHighPrivacy()

    v = triad_openvr.triad_openvr()
    start_point = v.devices["tracker_1"].get_pose_euler()


    if start_point is None:
        print("Cant read start data from sensor")
        while start_point is None:
            start_point = v.devices["tracker_1"].get_pose_euler()

    with open("z_angle_rotation.json", "r") as f:
        z_angle = json.load(f)['z_angle']

    # z_angle += pi

    start_point = np.array(start_point[:3])

    start_robot_pose = mcx.readCurrentPose()
    move_prev = None
    mcx.setSwith(1)
    # print(start_robot_pose)

    while True:
        try:
            current_point = v.devices["tracker_1"].get_pose_euler()
            if current_point is None:
                print("Cant read data from sensor", move)
                continue

            move = np.array(current_point[:3]) - start_point
            move = move / 2
            move = move.tolist()
            move[1], move[2] = move[2], move[1]

            move[1] = -move[1]

            move = z_rotation(move, z_angle)

            if move_prev is None:
                move_prev = move
            else:
                move = median_filter(move, move_prev)
                move_prev = move

            # print(move)

            move = mcx.checkAndSetPointConstrain(start_robot_pose, move)         

            mcx.writePoint(move)
            time.sleep(0.027)
        except KeyboardInterrupt:
            mcx.setSwith(0)
            mcx.writePoint([0, 0, 0])
            break

if __name__ == "__main__":
    main()
