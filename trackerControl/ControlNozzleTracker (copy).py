import motorcortex
import triad_openvr
import time
import numpy as np
import json
from math import *
import threading
from robot_control.motion_program import Waypoint, MotionProgram
from robot_control.robot_command import RobotCommand
from robot_control.system_defs import InterpreterStates
from serialLamp import SerialLamp

class MCX:
    def __init__(self, hostname):
        self.hostname = hostname
        self.inputCmd = 'root/Control/writingInput'
        self.swithCmd = 'root/Control/autoMotionGenerator/userFrames/switch01/toggle'
        self.currentPoseCmd = 'root/Control/actualToolCoordinates'
        self.ethercatParam = 'root/Ethercat/Robot/EL2024 4K. Dig. Ausgang 24V, 2A/Channel 1'
        self.buttonState = 'root/Control/dummyBool'
        self.robotState = "root/Logic/stateCommand"
        self.connect()

    def connect(self):
        parameter_tree = motorcortex.ParameterTree()
        self.messageTypes = motorcortex.MessageTypes()
        try:
            self.req, sub = motorcortex.connect("wss://" + self.hostname + ":5568:5567",
                                                self.messageTypes, parameter_tree,
                                                certificate="mcx.cert.pem", timeout_ms=1000,
                                                login="admin", password="vectioneer")
            
            self.robot = RobotCommand(self.req, self.messageTypes)
            
            if self.robot.engage():
                print('Robot is at Engage')
                # self.robot.reset()
                self.MoveToStartPoint()

                self.robot.play()
            else:
                raise Exception('Failed to set robot to Engage')

        except RuntimeError as err:
            print(err)
            exit()

    def getStartRobotPose(self):
        return self.start_robot_pose

    def MoveToStartPoint(self):
        motion_program = MotionProgram(self.req, self.messageTypes)    
        self.point = []
        self.startX = 0.40094
        self.startY = 0.15924   
        self.startZ = 0.53925
        self.point.append(Waypoint([self.startX, self.startY, self.startZ, radians(90), radians(0), radians(-180)]))
        motion_program.addMoveL(self.point, velocity=0.05, acceleration=0.1)
        
        motion_program.send("move_to_start_point").get() 
        self.robot.play()

        if self.robot.play() is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
            print('Robot is not at a start position, moving to the start')
            if self.robot.moveToStart(200):
                print('Robot is at the start position')
                time.sleep(2)
            else:
                raise Exception('Failed to move to the start position')

        print(self.robot.play())
            
        motion_program.send("move_to_start_point").get() 
        self.robot.play()
        
        # if self.robot.play() is InterpreterStates.PROGRAM_IS_DONE.value:
        #     self.start_robot_pose = self.readCurrentPose()
            
    def stopRobotProgram(self):
        self.robot.stop()

    def pauseRobotProgram(self):
        self.robot.pause()

    def robotPlay(self):
        self.robot.play()

    def readCurrentPose(self):
        get_param_reply_msg = self.req.getParameter(
            self.currentPoseCmd).get()
        current_pose = get_param_reply_msg.value
        return current_pose

    def setSwith(self, cmd):
        set_param_reply_msg = self.req.setParameter(
            self.swithCmd, cmd).get()
        
    def getState(self):
        return self.robot.getState()
    
    def writePoint(self, point: list):
        print("Writing point {0}".format(point))
        self.req.setParameter(
            self.inputCmd, point + [0, 0, 0]).get()
        
    def checkConstraine(self, point):
        startPose = [self.startX, self.startY, self.startZ]

        pose  = np.array(startPose[:3]) + np.array(point[:3])
        res_point = np.array(point)

        if pose[0] > float(self.x_sensor_control_range[1]):
            res_point[0] = float(self.x_sensor_control_range[1]) - startPose[0]

        if pose[0] < float(self.x_sensor_control_range[0]):
            res_point[0] = float(self.x_sensor_control_range[0]) - startPose[0]

        if pose[1] > float(self.y_sensor_control_range[1]):
            res_point[1] = float(self.y_sensor_control_range[1]) - startPose[1]

        if pose[1] < float(self.y_sensor_control_range[0]):
            res_point[1] = float(self.y_sensor_control_range[0]) - startPose[1]

        if pose[2] > float(self.z_sensor_control_range[1]):
            res_point[2] = float(self.z_sensor_control_range[1]) - startPose[2]

        if pose[2] < float(self.z_sensor_control_range[0]):
            res_point[2] = float(self.z_sensor_control_range[0]) - startPose[2]

        # print("Point", point)
        # print("Pose", self.manupulatorPose)
        # print("ResPoint", res_point.tolist())
              
        return res_point.tolist()

    def checkCriticalPose(self):
        with open("constraine.json", "r") as f:
            self.dataJSON = json.load(f)
            self.x_sensor_control_range = self.dataJSON['x_constraine']
            self.y_sensor_control_range = self.dataJSON['y_constraine']
            self.z_sensor_control_range = self.dataJSON['z_constraine']

        zOffset = 0.001
        self.manupulatorPose = self.readCurrentPose()

        if self.manupulatorPose[2] < float(self.z_sensor_control_range[0]) - zOffset:
            self.stopMotor()


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
              
        return res_point.tolist()

    def ethercatParamSet(self,):
        set_param_reply_msg = self.req.setParameter(
            self.ethercatParam, 1).get()
        
    def stopMotor(self):
        self.req.setParameter(self.robotState, 0).get()
        self.setSwith(0)
    
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

    def automatic(self):
        zValue = 0.162
        coordList = []
        start_pos = Waypoint([0.40094, 0.15924, 0.53925, radians(9.3), radians(0), radians(-180)])
        coordList.append(Waypoint([0.486, 0.362, 0.417, radians(24.428), radians(-0.2), radians(-180)]))
        coordList.append(Waypoint([0.440, -0.362, 0.417, radians(24.428), radians(-0.2), radians(-180)]))











        
        # coordList.append(Waypoint([0.06856, -0.47041, 0.24786, radians(95.83828480245795), radians(25.511858116418647), radians(180)]))
        # coordList.append(Waypoint([0.06856, -0.41995, 0.19726, radians(95.83828480245795), radians(25.511858116418647), radians(180)]))
        # coordList.append(Waypoint([0.30075, -0.41801, 0.19726, radians(95.83828480245795), radians(25.511858116418647), radians(180)]))
        # coordList.append(Waypoint([0.30075, -0.46506, 0.24786, radians(95.83828480245795), radians(25.511858116418647), radians(180)]))

        # coordList.append(Waypoint([0.34085, -0.31767, 0.247, radians(95.83828480245795), radians(25.511858116418647), radians(180)]))
        # coordList.append(Waypoint([0.34085, -0.31767, 0.180, radians(90), radians(0), radians(180)]))
        # coordList.append(Waypoint([0.34085, -0.31767, zValue, radians(90), radians(0), radians(180)]))
        # coordList.append(Waypoint([0.34085, -0.23144, zValue, radians(90), radians(0), radians(180)]))
        # coordList.append(Waypoint([0.26127, -0.23144, zValue, radians(90), radians(0), radians(180)]))
        # coordList.append(Waypoint([0.26127, -0.31767, zValue, radians(90), radians(0), radians(180)]))
        # coordList.append(Waypoint([0.34085, -0.31767, zValue, radians(90), radians(0), radians(180)]))
        motion_pr_1 = MotionProgram(self.req, self.messageTypes)
        motion_pr_1.addMoveL([start_pos], 0.1, 0.1)
        for i in range(len(coordList)):
            motion_pr_1.addMoveL([coordList[i]], 0.1, 0.1)

        motion_pr_1.send("Program1").get()
        self.robot.play()

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

    # v = triad_openvr.triad_openvr()
    # start_point = v.devices["tracker_1"].get_pose_euler()


    # if start_point is None:
    #     print("Cant read start data from sensor")
    #     while start_point is None:
    #         start_point = v.devices["tracker_1"].get_pose_euler()

    # with open("z_angle_rotation.json", "r") as f:
    #     z_angle = json.load(f)['z_angle']


    mcx = MCX('192.168.2.100')
    # mcx.ethercatParamSet()

    # mcx.automatic()

    # # z_angle += pi

    # start_point = np.array(start_point[:3])

    # key = 0

    # move_prev = None
    # move = [0,0,0]
    # mcx.setSwith(1)

    # while True:
    for i in range(3):
        mcx.automatic()
    #     mcx.checkCriticalPose()
    #     oldKey = key
    #     # print("Robot start:", mcx.getStartRobotPose())

    #     with open("switch_mode.json", "r") as f:
    #         key = json.load(f)['key']

    #     if key == oldKey:
    #         if key == 1:
    #             try:
    #                 # lamp.onGreen()
    #                 current_point = v.devices["tracker_1"].get_pose_euler()
    #                 if current_point is None:
    #                     print("Cant read data from sensor", move)
    #                     continue

    #                 move = np.array(current_point[:3]) - start_point
    #                 move = move / 3
    #                 move = move.tolist()
    #                 move[1], move[2] = move[2], move[1]

    #                 move[1] = -move[1]

    #                 move = z_rotation(move, z_angle)

    #                 if move_prev is None:
    #                     move_prev = move
    #                 else:
    #                     move = median_filter(move, move_prev)
    #                     move_prev = move

    #                 move = mcx.checkConstraine(move)

    #                 mcx.writePoint(move)
    #                 time.sleep(0.027)
    #             except KeyboardInterrupt:
    #                 mcx.setSwith(0)
    #                 mcx.writePoint([0, 0, 0])
    #                 break
    #         elif key == 2:
    #             errX = 0.001
    #             step = 0.0001

    #             if move[0] < errX and move[0] > -errX and move[1] < errX and move[1] > -errX and move[2] < errX and move[2] > -errX:
    #                 move[0] = 0
    #                 move[1] = 0
    #                 move[2] = 0
    #             else:
    #                 if move[0] < -errX: move[0] += step
    #                 if move[0] > errX: move[0] -= step
    #                 if move[1] < -errX: move[1] += step
    #                 if move[1] > errX: move[1] -= step
    #                 if move[2] < -errX: move[2] += step
    #                 if move[2] > errX: move[2] -= step

    #             mcx.writePoint(move)
    #             print(move)
    #             for i in range(3):
    #                 if move[i] == 0:
    #                     if mcx.getState() is InterpreterStates.PROGRAM_IS_DONE.value:
    #                         mcx.automatic()
    #     else:
    #         mcx.stopRobotProgram()
    #         mcx.MoveToStartPoint()
    #         mcx.robotPlay()

if __name__ == "__main__":
    main()