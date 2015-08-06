#!/usr/bin/env python

# Author: Tim Zimmerman (timothy.zimmerman@nist.gov)
# Organization: National Institute of Standards and Technology
# U.S. Department of Commerce
# License: Public Domain
#
# Name:         Youbot Modbus Server
#
# Description:  Created for use with ROS on the robotic enclave. This 
#               code will create a ROS node which polls a Modbus server
#               and provides the data it obtains to any other node via
#               the services it provides. This requires the Python package
#               'pymodbus' in order to operate. 

#               NOTE: rospy.signal_shutdown calls are required for raised
#                     exceptions, as we do not want the robots acting
#                     on old sensor/button data.
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
from pymodbus import bit_read_message, exceptions

station_states = []
button_states = []

plc_num_stations = 6
plc_num_buttons = 6

modbus_data_validity = True

def handle_station_request(data): 
    global station_states, modbus_data_validity
    # Build the ROS header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "youbot_modbus_station_response"
    return {'header'          : header,               \
            'data_validity'   : modbus_data_validity, \
            'station_1_status': station_states[0],    \
            'station_2_status': station_states[1],    \
            'station_3_status': station_states[2],    \
            'station_4_status': station_states[3],    \
            'station_5_status': station_states[4],    \
            'station_6_status': station_states[5]     }
    
def handle_button_request(data): 
    global button_states
    # Build the ROS header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "youbot_modbus_button_response"
    return {'header'                   : header,               \
            'data_validity'            : modbus_data_validity, \
            'estop_button_status'      : button_states[0],     \
            'estop_indicator_status'   : button_states[1],     \
            'yellow_button_status'     : button_states[2],     \
            'yellow_indicator_status'  : button_states[3],     \
            'green_button_status'      : button_states[4],     \
            'green_indicator_status'   : button_states[5],     \
            'runstop_switch_status'    : button_states[6],     \
            'runstop_indicator_status' : button_states[7],     \
            'supervisor_runstop_status': button_states[8]      }
    
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
    
def youbot_modbus_server_service():
    global station_states, button_states, modbus_data_validity
    connection_error_count = 0
    
    rospy.init_node('youbot_modbus_server')
    
    # Grab all of the parameters from the launch file
    mb_server_ip = rospy.get_param("~modbus_server_ip")
    mb_server_port = rospy.get_param("~modbus_server_port")
    mb_station_status_topic = rospy.get_param("~station_status_topic")
    mb_button_status_topic = rospy.get_param("~button_status_topic")
    plc_polling_rate = rospy.Rate(rospy.get_param("~plc_polling_rate_hz"))
    
    # Update the array sizes
    station_states = [False] * plc_num_stations
    button_states =  [False] * (plc_num_buttons * 2)
    
    # Create two services for nodes to call and get current sensor & button states
    s = rospy.Service(mb_station_status_topic, YoubotModbusSensorMsg, handle_station_request)
    t = rospy.Service(mb_button_status_topic, YoubotModbusButtonMsg, handle_button_request)
    
    # Attempt to connect to the PLC Modbus server
    try:
        # Connect to the Modbus Server
        client = ModbusClient(mb_server_ip, mb_server_port)
        if client.connect() == False:
            raise
    # We failed to connect to the PLC Modbus server. Report error and shutdown.
    except:
        except_desc = "Could not initiate connection with Modbus server"
        rospy.logerr(except_desc)
        rospy.signal_shutdown(except_desc)
    # We suceeded, so report the connection and continue
    else:
        rospy.loginfo("Youbot Modbus service successfully connected to: " + str(mb_server_ip) + ":" + str(mb_server_port))
        rospy.loginfo("Awaiting ROS service calls...")
    
    # Main loop for polling the Modbus Server on the PLC - plc_polling_rate param is in launch file
    while not rospy.is_shutdown():
        
        # Request data from the PLC Modbus server
        try:
            # Get the current station values (every two values is a ball/done pair)
            returned_station_states = client.read_discrete_inputs(0x8000, plc_num_stations*2)
            # Get the current button values (every two values is a button/indicator pair) (buttons follow stations in PLC addresses)
            returned_button_states = client.read_discrete_inputs((0x8000+(plc_num_stations*2)), plc_num_buttons*2)
            
            # Raise an exception if the server did not respond with the message type we were expecting
            if type(returned_station_states) != bit_read_message.ReadDiscreteInputsResponse or \
               type(returned_button_states ) != bit_read_message.ReadDiscreteInputsResponse:
                raise
            
            # If the data was invalid on a previous iteration, we need to report the transition
            if modbus_data_validity == False:
                connection_error_count = 0
                modbus_data_validity = True
                rospy.loginfo("PLC communication has been restored. Data validity: " + str(modbus_data_validity))
        
        # We did not get a reply from the PLC Modbus server, so count it and report a warning message.
        except exceptions.ConnectionException:
            modbus_data_validity = False
            connection_error_count += 1
            except_desc = "PLC did not respond to data request. Data validity: " + str(modbus_data_validity)
            rospy.logwarn(except_desc)
        
        # The PLC Modbus server did not respond with the type of message we were expecting.
        except:
            except_desc = "An exception was raised while attempting to read from the Modbus server"
            rospy.logerr(except_desc)
            rospy.signal_shutdown(except_desc)
        
        # Data is GOOD, so let's process it.
        else:
            # Report to all service calls that the data is GOOD
            modbus_data_validity = True
            # We got a good response, so update the arrays
            update_sensor_array(returned_station_states, plc_num_stations)
            update_button_array(returned_button_states, plc_num_buttons)
            
        # If we have not heard back from the PLC Modbus server in 
        # (connection_error_count * (1/plc_polling_rate) seconds, so we should shutdown.
        if connection_error_count >= 10:
            except_desc = "Could not communicate with PLC. Error count exceeded: "+ str(connection_error_count) + " failures"
            rospy.logerr(except_desc)
            rospy.signal_shutdown(except_desc)
            
        # Wait until the next Modbus poll
        plc_polling_rate.sleep()

if __name__ == "__main__":
    youbot_modbus_server_service()