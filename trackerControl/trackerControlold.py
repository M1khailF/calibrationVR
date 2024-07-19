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
import sys
import termios
import tty


class MCX:
    def __init__(self, hostname):
        self.hostname = hostname
        self.connect()
        self.inputCmd = 'root/Control/writingInput'
        self.swithCmd = 'root/Control/autoMotionGenerator/userFrames/switch01/toggle'
        self.currentPoseCmd = 'root/Control/actualToolCoordinates'
        self.ethercatParam = 'root/Ethercat/Robot/EL2024 4K. Dig. Ausgang 24V, 2A/Channel 1'
        self.buttonState = 'root/Control/dummyBool'
        self.robotState = "root/Logic/stateCommand"
        self.targetPosition = "root/Simulator/targetPosition"
        self.x_sensor_control_range = [0.25, 0.35]
        self.y_sensor_control_range = [-0.22, 0.22]
        self.z_sensor_control_range = [0.158, 0.25]

    def MoveToStartPoint(self):
        motion_program = MotionProgram(self.req, self.meesageTypes)    
        point = []
        x = 0.3
        y = 0.114
        z = 0.24
        point.append(Waypoint([x, y, z, radians(90), radians(0), radians(180)]))
        motion_program.addMoveL(point, velocity=0.05, acceleration=0.1)

        motion_program.send("move_to_start_point").get() 
        if self.robot.play() is InterpreterStates.MOTION_NOT_ALLOWED_S.value:
            print('Robot is not at a start position, moving to the start')
            if self.robot.moveToStart(200):
                print('Robot is at the start position')
                time.sleep(2)
            else:
                raise Exception('Failed to move to the start position')
            
    def connect(self):
        parameter_tree = motorcortex.ParameterTree()
        # Open request and subscribe connection
        self.meesageTypes = motorcortex.MessageTypes()
        try:
            self.req, sub = motorcortex.connect("wss://" + self.hostname + ":5568:5567",
                                                self.meesageTypes, parameter_tree,
                                                certificate="mcx.cert.pem", timeout_ms=1000,
                                                login="admin", password="vectioneer")
            
            self.robot = RobotCommand(self.req, self.meesageTypes)

            if self.robot.engage():
                print('Robot is at Engage')
                self.robot.reset()
                self.MoveToStartPoint()

                self.robot.play()
            else:
                raise Exception('Failed to set robot to Engage')

        except RuntimeError as err:
            print(err)
            # exit()

    def stopMotor(self):
        self.req.setParameter(
                        self.robotState, 0).get()
        
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
    
    def writePoint(self, point: list):
        print("Writing point {0}".format(point))
        self.req.setParameter(
            self.inputCmd, point + [0, 0, 0]).get()
        # self.req.setParameter(
        #     self.targetPosition, point).get()

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
    
    def automatic(self):
        # print(self.robot.getState())
        # while self.robot.getState() is InterpreterStates.PROGRAM_IS_DONE.value:
        #     time.sleep(0.1)
        #     print('Waiting for the program to start, robot state: {}'.format(self.req.getState()))
        mcx.writePoint([0, 0, 0])
        zValue = 0.158
        coordList = []
        start_pos = Waypoint([0.3, 0.114, 0.24, radians(90), radians(0), radians(180)])
        coordList.append(Waypoint([0.25, -0.15, zValue, radians(90), radians(0), radians(180)]))
        coordList.append(Waypoint([0.33, -0.15, zValue, radians(90), radians(0), radians(180)]))
        coordList.append(Waypoint([0.33, 0.15, zValue, radians(90), radians(0), radians(180)]))
        coordList.append(Waypoint([0.25, 0.15, zValue, radians(90), radians(0), radians(180)]))
        coordList.append(Waypoint([0.25, -0.15, zValue, radians(90), radians(0), radians(180)]))

        # point to point move
        motion_pr_1 = MotionProgram(self.req, self.meesageTypes)
        motion_pr_1.addMoveL([start_pos], 0.1, 0.1)
        for i in range(len(coordList)):
            motion_pr_1.addMoveL([coordList[i]], 0.2, 0.2)

        motion_pr_1.send("Program1").get()
        self.robot.play()
        # while self.robot.getState() is InterpreterStates.PROGRAM_RUN_S.value:
        #     time.sleep(0.1)
        #     print('Playing, robot state: {}'.format(self.req.getState()))

    def ethercatParamSet(self,):
        set_param_reply_msg = self.req.setParameter(
            self.ethercatParam, 1).get()
    
    def getState(self):
        return self.robot.getState()

    def getParam(self):
        return self.req, self.meesageTypes, self.robot
    
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
        # threading.Thread(target=checkAndStopCallback, daemon=True).start()

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

def keyboard_input():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        
    return ch

key = 1
flag = True

    # robot.play()

def btnPress():
    global key, flag, mcx, keyboard_thread
    bntListExit = ['q', 'Q', 'й', 'Й']
    bntListAutoMode = ['a', 'A', 'ф', 'Ф']
    bntListTraker = ['t', 'T', 'е', 'Е']
    while True:
        char = keyboard_input()
        print(f"Pressed: {char}")
        if char in bntListTraker:
           key = 1
           flag = False
           mcx.pauseRobotProgram()
           mcx.stopRobotProgram()
           mcx.MoveToStartPoint()

        if char in bntListAutoMode:
           key = 2
           flag = False
           mcx.pauseRobotProgram()
           mcx.stopRobotProgram()
           mcx.MoveToStartPoint()
           mcx.robotPlay()

        if char in bntListExit:
           key = -1
           mcx.stopMotor()
           exit()


def input_keyboard():
    char = input()
    return char

def main():
    global key, flag, mcx, keyboard_thread

    manipulatorIP = '192.168.2.100'
    mcx = MCX(manipulatorIP)

    mcx.ethercatParamSet()
    mcx.checkButtonWithHighPrivacy()
    mcx.MoveToStartPoint()
    # threading.Thread(target=btnPress, daemon=True).start()

    # keyboard_thread = threading.Thread(target=btnPress)
    # keyboard_thread.daemon = True  # Поток будет работать в фоновом режиме
    # keyboard_thread.start()
    # move = [0,0,0]
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


    oldKey = 0

    # print("then")

    while True:
        oldKey = key
        with open("z_angle_rotation.json", "r") as f:
            key = json.load(f)['key']

        print(key)
        if key == oldKey:
            if flag == True:
                # req, mcxtypes, robot = mcx.getParam()
                if key == 1 and mcx.getState() == 200:
                    # print(flag, key)
                    # print(mcx.getState())
                    # print(robot.getState())
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

                        print(move)

                        move = mcx.checkAndSetPointConstrain(start_robot_pose, move)         

                        mcx.writePoint(move)
                        time.sleep(0.027)
                    except KeyboardInterrupt:
                        mcx.setSwith(0)
                        mcx.writePoint([0, 0, 0])
                        # mcx.writePoint(move)
                        # start_point = move
                        break
                elif key == 2:
                    # print(flag, key)
                    # errX = 0.01
                    # if move[0] < -errX: move[0] += 0.01
                    # if move[0] > errX: move[0] -= 0.01
                    # if move[1] < -errX: move[1] += 0.01
                    # if move[1] > errX: move[1] -= 0.01
                    # if move[2] < -errX: move[2] += 0.01
                    # if move[2] > errX: move[2] -= 0.01

                    # if move[0] < errX and move[0] > -errX and move[1] < errX and move[1] > -errX and move[2] < errX and move[2] > -errX:
                    #     mcx.writePoint([0, 0, 0])

                    if mcx.getState() is InterpreterStates.PROGRAM_IS_DONE.value:
                        # mcx.stopRobotProgram()
                        # mcx.MoveToStartPoint()
                        mcx.automatic()
                    # automatic(req, mcxtypes, robot)
                    # print(robot.getState())
                    # pass

                elif key == 0:
                    mcx.stopMotor()
                    exit()
                else:
                    if mcx.getState() == 2:
                        # mcx.robotPlay()
                        # keyboard_thread.start()
                        mcx.robotPlay()
                    else:
                        print("Wait when program was done")
            else:
                mcx.MoveToStartPoint()
                # print(flag, 0)
                flag = True
                # print(flag, 1)
        else:
            # mcx.pauseRobotProgram()
            mcx.stopRobotProgram()
            mcx.MoveToStartPoint()
            mcx.robotPlay()

if __name__ == "__main__":
    main()
