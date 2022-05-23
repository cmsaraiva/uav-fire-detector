from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import datetime
import os

# Set up option parsing to get connection string
import argparse  

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# setting up file for log
now = datetime.datetime.now()
filename = str(now) + ".txt"
f = open(filename, "a")

def logDebug(message):
	print(" [DEBUG]: {0}".format(message), file=f)	

def logInfo(message):
	print(" [INFO]: {0}".format(message), file=f)

def verifySystem():

	logDebug ("Starting system analysis")
	logInfo (" Global Location: %s" % vehicle.location.global_frame)
	logInfo (" Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
	logInfo (" Local Location: %s" % vehicle.location.local_frame)
	logInfo (" Attitude: %s" % vehicle.attitude)
	logInfo (" Velocity: %s" % vehicle.velocity)
	logInfo (" GPS: %s" % vehicle.gps_0)
	logInfo (" Gimbal status: %s" % vehicle.gimbal)
	logInfo (" Battery: %s" % vehicle.battery)
	logInfo (" EKF OK?: %s" % vehicle.ekf_ok)
	logInfo (" Last Heartbeat: %s" % vehicle.last_heartbeat)
	logInfo (" Rangefinder: %s" % vehicle.rangefinder)
	logInfo (" Rangefinder distance: %s" % vehicle.rangefinder.distance)
	logInfo (" Rangefinder voltage: %s" % vehicle.rangefinder.voltage)
	logInfo (" Heading: %s" % vehicle.heading)
	logInfo (" Is Armable?: %s" % vehicle.is_armable)
	logInfo (" System status: %s" % vehicle.system_status.state)
	logInfo (" Groundspeed: %s" % vehicle.groundspeed)    # settable
	logInfo (" Airspeed: %s" % vehicle.airspeed)    # settable
	logInfo (" Mode: %s" % vehicle.mode.name)    # settable
	logInfo (" Armed: %s" % vehicle.armed)    # settable
	
	
	if vehicle.battery.level != 100 :
		 logInfo (" Vehicle not fully charged")
		 return False

	while not vehicle.ekf_ok:
		logInfo (" Vehicle EKF not ok")
		logDebug (" Waiting EKF")
		time.sleep(2)
		 
	while not vehicle.is_armable:
		logInfo (" Vehicle is not Armable")
		logDebug (" Waiting to be armable...")
		time.sleep(1)
	
	if vehicle.system_status.state != "STANDBY" :
		logInfo (" Vehicle system not ok")
		return False
		 
	return True

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  logDebug (" Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    logDebug (" Waiting for arming...")
    time.sleep(1)

  logDebug (" Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    logDebug (" Altitude: %s" % vehicle.location.global_relative_frame.alt )
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      logDebug ("Reached target altitude")
      break
    time.sleep(1)

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def adds_square_mission(aLocation, rWidth, rHeigth):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).
    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	
    cmds = vehicle.commands

    logDebug(" Clear any existing commands")
    cmds.clear() 
    
    logDebug(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 120))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, 5, 0)
    point2 = get_location_metres(aLocation, 5+rHeigth, 0)
    point3 = get_location_metres(aLocation, 5+2*rHeigth, 0)
    point4 = get_location_metres(aLocation, 5+3*rHeigth, 0)
    point5 = get_location_metres(aLocation, 10+3*rHeigth, rWidth)
    point6 = get_location_metres(aLocation, 5+3*rHeigth, rWidth)
    point7 = get_location_metres(aLocation, 5+2*rHeigth, rWidth)
    point8 = get_location_metres(aLocation, 5+rHeigth, rWidth)
    point9 = get_location_metres(aLocation, 5, rWidth)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, point1.lat, point1.lon, 120))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, point2.lat, point2.lon, 120))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, point3.lat, point3.lon, 120))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, point4.lat, point4.lon, 120)) 
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point5.lat, point5.lon, 120))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, point6.lat, point6.lon, 120))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, point7.lat, point7.lon, 120))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, point8.lat, point8.lon, 120))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 10, 0, 0, 0, point9.lat, point9.lon, 120))	
	
    #add dummy waypoint "10" (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point9.lat, point9.lon, 120))    

    logDebug(" Uploading new commands to vehicle")
    cmds.upload()
	
def fire_detection() :
	return False

# Connect to the Vehicle
logInfo ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)
logInfo ('Connected')


# Verify system_status
if  verifySystem() == True :

	logInfo(' Create a new mission (for current location)')
	adds_square_mission(vehicle.location.global_frame,251, 141)

	# Initialize the takeoff sequence to 100m
	logDebug(" Starting takeoff")
	arm_and_takeoff(120)
	logInfo(" Take off complete")

	logInfo("Starting mission")
	# Reset mission set to first (0) waypoint
	vehicle.commands.next=0

	# Set mode to AUTO to start mission
	vehicle.mode = VehicleMode("AUTO")

	# Monitor mission. 
	# Demonstrates getting and setting the command number 
	# Uses distance_to_current_waypoint(), a convenience function for finding the 
	#   distance to the next waypoint.

	while True:
		nextwaypoint=vehicle.commands.next
		logDebug('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
		if(nextwaypoint!=5 and nextwaypoint!=9 and distance_to_current_waypoint() < 3):
			lat = vehicle.location.global_frame.lat
			lon = vehicle.location.global_frame.lon
			alt = vehicle.location.global_frame.alt
			runCmd = "python fireDetector2.py " + str(lat) + " " + str(lon) + " " + str(alt)
			logInfo("Running fire dection script at lat=%s lon=%s alt=%s" % (lat,lon,alt) )
			os.system(runCmd)
			time.sleep(10)
		if nextwaypoint==10: #Dummy waypoint 
			logInfo("Exit 'standard' mission")
			break;
		time.sleep(0.5)
	
	# Set vehicle to return to base
	logInfo('Return to launch')
	vehicle.mode = VehicleMode("RTL")	
else:
	logInfo("System not ok for takeoff, aborting mission")

#Close vehicle object before exiting script
logDebug("Close vehicle object")
vehicle.close()
f.close()
os.rename( filename , "logs/" + filename)

