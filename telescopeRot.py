
import time
import win32com.client
# import pythoncom
from datetime import datetime
import numpy as np
# import ephem
import csv
import os
current_Azimuth = 0
current_Altitude = 0
# Create logs directory if it doesn't exist
os.makedirs("logs", exist_ok=True)
# Create logs directory if it doesn't exist
# Create logs directory if it doesn't exist
log_folder_path = "C:/Users/IceCube/OneDrive - University of Utah/Desktop/Telescope/logs"
os.makedirs(log_folder_path, exist_ok=True)

# Generate a unique log filename using current date and time
log_filename = f"telescope_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
full_log_file_path = os.path.join(log_folder_path, log_filename)

# Write CSV headers
with open(full_log_file_path, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp Start", "Timestamp End", "Azimuth (deg)", "Altitude (deg)", "Gamma (deg)"])


# Generate a unique log filename using current date and time
log_filename = f"logs/telescope_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# Write CSV headers
with open(log_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Starting Timestamp","Ending Timestamp", "Azimuth (deg)", "Altitude (deg)", "Gamma (deg)"])

# Connect to ASCOM telescope
telescope = win32com.client.Dispatch("ASCOM.Celestron.Telescope")
telescope.Connected = True
# telescope.SlewToAltAz(telescope.Azimuth-90, telescope.Altitude)

def azalt_to_cartesian(azimuth, altitude):
   x = np.cos(np.radians(altitude)) * np.cos(np.radians(azimuth))
   y = np.cos(np.radians(altitude)) * np.sin(np.radians(azimuth))
   z = np.sin(np.radians(altitude))
   return np.array([x, y, z])

def cartesian_to_azalt(cartesian_vector):
    x, y, z = cartesian_vector
    azimuth = np.degrees(np.arctan2(y, x))
    altitude = np.degrees(np.arcsin(z))
    return azimuth, altitude

def rotate_vector(v, axis, mu_degrees):
    # Convert angle from degrees to radians
    v = np.array(v, dtype=float)
    axis = np.array(axis, dtype=float)    
    mu = np.radians(mu_degrees)
    
    # Normalize the rotation axis
    axis = axis / np.linalg.norm(axis)
    
    # Rodrigues' rotation formula
    v_rot = (v * np.cos(mu) +
             np.cross(axis, v) * np.sin(mu) +
             axis * np.dot(axis, v) * (1 - np.cos(mu)))

    # print(f"Rotated vector: {cartesian_to_azalt(v_rot)}")
    # Return the rotated vector
    return v_rot

def MoveTelescope(axis, angle):
 
  if axis=="Azimuth":
    print(f"moving in Azimuth by {angle}")
    if angle>0:
      telescope.MoveAxis(0, 2)  # Move the telescope in azimuth at 2 deg/sec
    else:
      telescope.MoveAxis(0, -2)  # Move the telescope in azimuth at -2 deg/sec
    time.sleep(float(abs(angle)/2.0))  # Wait for the movement to complete
    telescope.MoveAxis(0, 0)
    # current_Azimuth = current_Azimuth + angle
    
  elif axis=="Altitude":
    print(f"moving in Altitude by {angle}")
    if angle>0:
      telescope.MoveAxis(1, 2)  # Move the telescope to a known position
    else:
      telescope.MoveAxis(1, -2)  # Move the telescope to a known position
    time.sleep(float(abs(angle)/2.0))  # Wait for the movement to complete
    telescope.MoveAxis(1, 0)
    # current_Altitude = current_Altitude + angle


# MoveTelescope('Azimuth', 90)

current_Azimuth = 0
current_Altitude = 0
while True:

  Test_Question = input('Would You like to run test? (y,n): ')
  if Test_Question == 'n':
    while True:
        input_Azimuth = float(input('Go to Azimuth: '))
        input_Altitude = float(input('Go to Altitude: '))
        move_Azimuth = input_Azimuth - current_Azimuth
        move_Altitude = input_Altitude - current_Altitude
        MoveTelescope('Azimuth', move_Azimuth)
        MoveTelescope('Altitude', move_Altitude)
        current_Azimuth = input_Azimuth
        current_Altitude = input_Altitude
  else:
    degrees_below_sun = float(input('How many degrees below sun would you like to do?: '))
    t = float(input('How long do you want to integrate power (s): '))
    #  sun_Azimuth = float(input('What is the Azimuth of the sun: '))
    #  sun_Altitude = float(input('What is the Altitude of the sun: '))
    #  MoveTelescope('Azimuth', sun_Azimuth)
    #  MoveTelescope('Altitude', sun_Altitude)
    #  current_Azimuth = sun_Azimuth
    #  current_Altitude = sun_Altitude
    
    #  list_of_altitudes_below = [i for i in np.arange(0, 23.5 + 1, 4)]
    list_of_altitudes_below = [degrees_below_sun]
    for altDiff in list_of_altitudes_below:
      print(f'We are going {altDiff} degrees below the sun')
      sun_Azimuth = float(input('What is the Azimuth of the sun: '))
      sun_Altitude = float(input('What is the Altitude of the sun: '))
      alt = sun_Altitude - altDiff
      current_azimuth = 0  # or wherever the telescope starts
      MoveTelescope('Altitude', alt)
      MoveTelescope('Azimuth', sun_Azimuth)


      current_Azimuth = sun_Azimuth
      current_Altitude = alt
      Part_Three_AzAlts = []
      panel_vector = azalt_to_cartesian(sun_Azimuth, alt)
      panel_Azimuth, panel_Altitude = cartesian_to_azalt(panel_vector)
      rotation_Axis = azalt_to_cartesian(panel_Azimuth + 180, 90 - panel_Altitude)
      # Makes the list of vectors to test
      for i in [0,30,60,90,120,150,180]:
          az, alt = cartesian_to_azalt(rotate_vector(panel_vector, rotation_Axis, i))
          if az < 0:
            az = az +360
          Part_Three_AzAlts.append((az, alt))
          print(f'on GC: {i}')
          # print(f'This is the place to go: {cartesian_to_azalt(rotate_vector(panel_vector, rotation_Axis, i))}')
      # Tests each vector
      print(f"Part_Three_AzAlts:{Part_Three_AzAlts}")
      for great_circ_count, i in enumerate(Part_Three_AzAlts):
          print(f'At {great_circ_count*30} degrees of great circle')
          print(i)
          MoveTelescope('Altitude', i[1] - current_Altitude)
          MoveTelescope('Azimuth', i[0] - current_Azimuth)
          print(f'Testing vector: moving to: (Az,Alt)={i}')
          data_start_time=datetime.now().strftime('%H:%M:%S')
          print(f"Data Start at {data_start_time}")
          polar_sun_altitude = 90 - sun_Altitude
          polar_panel_altitude = 90 - i[1]
          gamma = np.degrees(np.arccos(
          np.sin(np.radians(polar_panel_altitude)) *
          np.sin(np.radians(polar_sun_altitude)) *
          np.cos(np.radians(i[0] - sun_Azimuth)) +
          np.cos(np.radians(polar_panel_altitude)) *
          np.cos(np.radians(polar_sun_altitude))
          ))
          
          print(f'full: {np.sin(np.radians(polar_panel_altitude))*np.sin(np.radians(polar_sun_altitude))*np.cos(np.radians(i[0] - sun_Azimuth))+np.cos(np.radians(polar_panel_altitude)*np.cos(np.radians(polar_sun_altitude)))}')
          print(f'Gamma = {gamma}')
          # a = ephem.separation((current_Azimuth, current_Altitude), (sun_Azimuth, sun_Altitude))
          # print(a)
          print(f'panel vector: {i}')
          current_Azimuth = i[0]
          current_Altitude = i[1]
          time.sleep(t)
          data_end_time=datetime.now().strftime('%H:%M:%S')
          print(f"Data End at {data_end_time}")
          with open(full_log_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                data_start_time,
                data_end_time,
                round(current_Azimuth, 4),
                round(current_Altitude, 4),
                round(gamma, 4)
            ])
            print()


      print('returning to index')
      print(f'Current altitude: {current_Altitude}')
      print(f'Current azimuth: {current_Azimuth}')
      MoveTelescope('Altitude', -current_Altitude)
      MoveTelescope('Altitude', 30)
      MoveTelescope('Azimuth', -current_Azimuth)
      MoveTelescope('Altitude', -30)
      current_Azimuth = 0
      current_Altitude = 0
      print(alt)

          

    #  Altitudes_Below_Sun = [sun_Altitude - i for i in np.arange(0, 23.5 + 1, 10)]
    #  for Alt in Altitudes_Below_Sun:
    #     Part_Three_AzAlts = []
