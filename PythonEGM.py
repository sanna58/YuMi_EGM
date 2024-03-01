# -*- coding: utf-8 -*-

import egm_pb2 as egm
import socket
import asyncio
import websockets
import json
from PyhtonRWS import RWPanel

computer_ip = "192.168.125.203" #For use on physical controller (need static IP on a local network)
# computer_ip= "10.245.0.242" #For simulation in robotstudio - Same as in robotstudio-configuration-communication
robot_port=6512 # Port of ROB_R mechanical unit

num=0
position_data = None

uri = "ws://10.245.1.25:9090" #For ROS Socket from websocket in Linux running Isaac sim
topic = "/joint_states1"  # ROS Topic

# RWS author details
host = 'localhost:80' 
username = 'Default User'
password = 'robotics'
rwpanel = RWPanel(host, username, password)
routine_status = '0'

robot_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #Sets up a client to receive UDP messages
robot_socket.bind((computer_ip, robot_port)) #Binds the client to listen on your IP and port (same as specified in Controller-Configuration-

async def main():
    rwpanel.set_EGM_START_JOINT()

    loop_task = asyncio.get_event_loop().run_in_executor(None, run_egm_communication, num, position_data) # Start the EGM communication loop
    
    ros_task = asyncio.create_task(subscribe_to_ros_topic()) # Start the ROS subscription
    
    await asyncio.gather(loop_task, ros_task) # Wait for both tasks to complete

def CreateSensorMessage(egmSensor, pos, quat, position_data):
    headerOne=egmSensor.header
    headerOne.seqno=num
    headerOne.mtype=egm.EgmHeader.MessageType.MSGTYPE_CORRECTION    
    
    #to change the position and/or orientation of the robot, change values of input vectors
    planned=egmSensor.planned
    speedref=egmSensor.speedRef
    joint=planned.joints
    pose=planned.cartesian
    
    joint_angles2=[position_data[1], position_data[3], position_data[7], position_data[9], position_data[11], position_data[13]]
    joint_angles_ext=[position_data[5]]
    
    planned.joints.joints.extend(joint_angles2)
    planned.externalJoints.joints.extend(joint_angles_ext)
    return egmSensor

print(f"Listening on {computer_ip}:{robot_port}")

async def subscribe_to_ros_topic():
    subscribe_message = {
        "op": "subscribe",
        "topic": topic,
        "type": "sensor_msgs/JointState" 
    }

    async with websockets.connect(uri) as websocket:
        # Send the subscription message
        await websocket.send(json.dumps(subscribe_message))
        print(f"Subscribed to topic: {topic}")
        routine_status = '0'
        try:
            while True:
                data = await websocket.recv() # Receive data from the websocket
                parsed_data = json.loads(data) # Parse and print the received message
                seq_value = parsed_data['msg']['header']['seq']
                position_data = parsed_data['msg']['position']                
                # print(f"Seq: {seq_value}")
                print(f"Position: {position_data}")

                if position_data[16] + position_data[17] > 0.025 and routine_status == '0': # If the gripper is opened more than the size of slide
                    rwpanel.set_RUN_SG_ROUTINE_DI('1') # Turn on the Digital Input --> add GRIP_OUT command_input in rapid for HIGH signal from RUN_SG_ROUTINE_DI
                    routine_status = '1'

                if position_data[16] + position_data[17] < 0.025 and routine_status == '1': # If the gripper is opened more than the size of slide
                    rwpanel.set_RUN_SG_ROUTINE_DI('0') # Turn off the Digital Input --> add GRIP_in command_input in rapid for LOW signal from RUN_SG_ROUTINE_DI
                    routine_status = '0'

                run_egm_communication(seq_value, position_data)

        except websockets.exceptions.ConnectionClosedOK:
            print("Connection closed cleanly.")

def run_egm_communication(num, position_data):
    # while True:
        # print(f"Position: {position_data}")
        data, addr = robot_socket.recvfrom(1024)  # Buffer size is 1024 byte
        
        if data is not None and position_data is not None: # Uncomment for including isaac-sim position data stream            
            print(f"Seq: {num}")
            # print(f"Position: {position_data}")
            # print(f"Received message from {addr}")

            message=egm.EgmRobot() #Reads-in and deserializes the protocol buffer message from controller
            message.ParseFromString(data)
                        
            Seq=message.header.seqno
            Time=message.header.tm
            CurX=message.feedBack.cartesian.pos.x
            CurY=message.feedBack.cartesian.pos.y
            CurZ=message.feedBack.cartesian.pos.z
            feedBack_joints_R =message.feedBack.joints.joints
            feedBack_joints_R_external =message.feedBack.externalJoints.joints
            Updated_joints = []
            for i in range(len(position_data)):
                Updated_joints.append(position_data[i] * (180/3.14))

            print("The 6 joints are:", feedBack_joints_R)
            print("The external joints joints are:", feedBack_joints_R_external)
            
            ###Setup for message back to Robot Controller (see readme and EGM manual for specifics)####
            Pos=[50,-250,30] #[x,y,z] choords
            Quat=[1,0,0,0] #[q0,q1,q2,q3] quaternion
            egmSensor=egm.EgmSensor()
            egmSensor=CreateSensorMessage(egmSensor,Pos,Quat, Updated_joints)

            #To Serialize with protocol buffer and transmit message to Controller (either message type)
            mess=egmSensor.SerializeToString()
            robot_socket.sendto(mess, addr)

            sent_successfully = False
            max_attempts = 5
            attempt = 0

            while not sent_successfully and attempt < max_attempts:
                try:
                    # robot_socket.sendto(mess, addr)
                    sent_successfully = True
                except socket.error as e:
                    print(f"Sending failed: {e}. Retrying...")
                    attempt += 1

            if not sent_successfully:
                print("Warning: Message could not be sent after multiple attempts.")
            
if __name__ == "__main__":
    asyncio.run(main())
