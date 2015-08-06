#!/usr/bin/env python

# Author: Tim Zimmerman (timothy.zimmerman@nist.gov)
# Organization: National Institute of Standards and Technology
# U.S. Department of Commerce
# License: Public Domain
#
# Description:  Created for use with ROS on the robotic enclave. This 
#               code will create a ROS node which polls a Modbus server
#               and provides the data it obtains to any other node via
#               the services it provides. This requires the Python package
#               'pymodbus' in order to operate.
#
# Params:       ~modbus_server_ip
#               ~modbus_server_port
#               ~station_status_topic
#               ~button_status_topic
#               ~plc_polling_rate_hz
#         

import rospy
import std_msgs.msg
from youbot_modbus_server.srv import *
from pymodbus.client.sync import ModbusTcpClient as ModbusClient

station_states = []
button_states = []

plc_num_stations = 6
plc_num_buttons = 6

def handle_station_request(data): 
    global station_states
    # Build the ROS header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "youbot_modbus_station_response"
    return YoubotModbusSensorMsgResponse( \
        header,station_states[0],station_states[1],station_states[2], \
        station_states[3],station_states[4],station_states[5])
    
def handle_button_request(data): 
    global button_states
    # Build the ROS header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "youbot_modbus_button_response"
    return YoubotModbusButtonMsgResponse( \
        header,button_states[0],button_states[1],button_states[2], \
        button_states[3],button_states[4],button_states[5], \
        button_states[6],button_states[7],button_states[8])
    
def update_sensor_array(data, num_stations): 
    i = 0
    return_array = []
    while i < num_stations * 2:
       if data.bits[i] == True and data.bits[i+1] == False:
           return_array.append(1) # Station is currently processing
       elif data.bits[i] == True and data.bits[i+1] == True:
           return_array.append(2) # Station is awaiting pickup
       else:
           return_array.append(0) # Station is empty
       i = i + 2
    return return_array
    
def update_button_array(data, num_buttons): 
    return_array = []
    for i in range(len(button_states)):
        return_array.append(data.bits[i])
    return return_array
    
def youbot_modbus_server_server(): 
    global station_states
    global button_states
    
    rospy.init_node('youbot_modbus_server')
    
    # Grab all of the parameters from the launch file
    mb_server_ip = rospy.get_param("~modbus_server_ip")
    mb_server_port = rospy.get_param("~modbus_server_port")
    mb_station_status_topic = rospy.get_param("~station_status_topic")
    mb_button_status_topic = rospy.get_param("~button_status_topic")
    plc_polling_rate = rospy.Rate(rospy.get_param("~plc_polling_rate_hz"))
    
    # Update the array sizes
    station_states = [False] * plc_num_stations
    button_states = [False] * (plc_num_buttons * 2)
    
    # Create two services for nodes to call and get current sensor & button states
    s = rospy.Service(mb_station_status_topic, YoubotModbusSensorMsg, handle_station_request)
    t = rospy.Service(mb_button_status_topic, YoubotModbusButtonMsg, handle_button_request)
    
    # Connect to the Modbus Server
    client = ModbusClient(mb_server_ip, mb_server_port)
    client.connect()
    
    rospy.loginfo("Youbot Modbus Server is listening for requests...")
    
    # Main loop for polling the Modbus Server on the PLC - plc_polling_rate param is in launch file
    while not rospy.is_shutdown():
        # Get the current station values (every two values is a ball/done pair)
        station_states = update_sensor_array(client.read_discrete_inputs(0x8000, plc_num_stations*2), plc_num_stations)
        # Get the current button values (every two values is a button/indicator pair) (buttons follow stations in PLC addresses)
        button_states = update_button_array(client.read_discrete_inputs((0x8000+(plc_num_stations*2)), plc_num_buttons*2), plc_num_buttons)
        # Wait until the next Modbus poll
        plc_polling_rate.sleep()

if __name__ == "__main__":
    youbot_modbus_server_server()