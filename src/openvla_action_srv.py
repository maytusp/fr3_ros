#!/usr/bin/python3

# For CONDA Env: /usr/bin/env python 
import glob
import pickle
import math
import os, sys
import time
import rospy
from cv_bridge import CvBridge
import socket
import struct

from openvla_real_expr.srv import openvla_action_srv,openvla_instr_srv, openvla_action_srvResponse

# import sys
# sys.path.insert(0,'/home/rishabh/Robot/')

global openvlaWrapper
global s
global time_step
global task

def handle_instruction(req):
    global openvlaWrapper
    global s
    global time_step
    global task 

    HOST = socket.gethostbyname(socket.gethostname())  # The server's hostname or IP address
    PORT = 7670  # The port used by the server
    time_step = 0
    s = socket.socket()
    s.connect((HOST, PORT))

    task = req.instruction
    print("task:", task)

    return []

def handle_action(req):
    global openvlaWrapper
    global s
    global time_step
    global task 
    mode = 1 #TODO Add mode here and send to cpp script 1 is using pose, 0 is using delta pose
    image = req.image

    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough') # (720, 1280, 3)
    # frame = np.expand_dims(frame, axis=0) # (1, 720, 1280, 3)
    # print("frame value", frame)

    # frame = np.array(frame)
    # while time_step<100:
    data_send = pickle.dumps({"time_step": time_step, "frame": frame, "task": task})
    s.sendall(struct.pack('L', len(data_send)))
    s.sendall(data_send)
    data_rec = s.recv(4096)
    data_rec = pickle.loads(data_rec)
    action = list(data_rec['action'])
    # print(f"Timestep: {time_step}")
    # print(f"Action {data_rec!r}")
    # time_step+=1

    response = openvla_action_srvResponse()
    response.action = action

    print("response action is :", response.action)

    return response

if __name__=='__main__':
    
    rospy.init_node('openvla_action_srv')
    rospy.Service('/openvla_real_expr/openvla_instr', openvla_instr_srv,  handle_instruction)
    rospy.Service('/openvla_real_expr/openvla_action', openvla_action_srv,  handle_action)

    print("[Ready]")
    rospy.spin()