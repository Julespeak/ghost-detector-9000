import neopixel
import board
import RPi.GPIO as GPIO
import socket
import numpy as np
import math
import time
from numpy import linalg as LA
import pygame

HOST = "ip_address"  # Ghost detector IP address
PORT = 9005  # SocketHost listening port

sensor_adjustment_matrix = np.array(
	[[0.0, -1.0, 0.0],
	[1.0, 0.0, 0.0],
	[0.0, 0.0, 1.0]]
)

###########
# Functions

# version i trust a little more from here: http://www.songho.ca/opengl/gl_quaternion.html
def quaternion_rotation_matrix(Q):
	# Extract the values from Q
	q0 = Q[0]
	q1 = Q[1]
	q2 = Q[2]
	q3 = Q[3]

	r00 = 1 - 2*q2**2 - 2*q3**2
	r01 = 2*q1*q2 - 2*q0*q3
	r02 = 2*q1*q3 + 2*q0*q2

	r10 = 2*q1*q2 + 2*q0*q3
	r11 = 1 - 2*q1**2 - 2*q3**2
	r12 = 2*q2*q3 - 2*q0*q1

	r20 = 2*q1*q3 - 2*q0*q2
	r21 = 2*q2*q3 + 2*q0*q1
	r22 = 1 - 2*q1**2 - 2*q2**2

	# 3x3 rotation matrix
	rot_matrix = np.array([[r00, r01, r02],
						   [r10, r11, r12],
						   [r20, r21, r22]])

	return rot_matrix

def add_ghost_to_array(ghost_array):
	if len(ghost_array) < 5: # Set a cap on the allowed ghost number
		#Pick a random point somewhere in the allowed space
		ghost_theta = np.random.uniform(np.pi/4., np.pi/2.)
		ghost_phi = np.random.uniform(0, 2*np.pi)
		ghost_distance = 30.0

		ghost_x_coord = ghost_distance * np.sin(ghost_theta) * np.cos(ghost_phi)
		ghost_y_coord = ghost_distance * np.sin(ghost_theta) * np.sin(ghost_phi)
		ghost_z_coord = ghost_distance * np.cos(ghost_theta)

		ghost_array.append([ghost_x_coord, ghost_y_coord, ghost_z_coord])

	return ghost_array

def detect_ghosts(quat, ghost_array, orientation_matrix):
	detected_ghosts = []

	# Get a rotation matrix from the current quaternion
	rot_matrix = quaternion_rotation_matrix(quat)

	# Rotate about z axis to correct for sensor orientation
	adjusted_rot_matrix = np.matmul(sensor_adjustment_matrix, rot_matrix)
	
	# Apply rotation based on last calibrated position
	oriented_rot_matrix = np.matmul(adjusted_rot_matrix, orientation_matrix) #Rotation relative to initial heading
	
	current_heading = np.matmul(oriented_rot_matrix, np.array([1, 0, 0]))
	# print("Current heading: (%5.4f, %5.4f, %5.4f)"%(current_heading[0], current_heading[1], current_heading[2]))

	# Get the current theta and phi angles for the detector's orientation;
	#  taken from https://stackoverflow.com/questions/4116658/faster-numpy-cartesian-to-spherical-coordinate-conversion
	xy = current_heading[0]**2 + current_heading[1]**2
	detector_r = np.sqrt(xy + current_heading[2]**2)
	detector_theta = np.arctan2(current_heading[2], np.sqrt(xy)) # for elevation angle defined from Z-axis down
	detector_phi = np.arctan2(current_heading[1], current_heading[0])

	# Iterate through all ghosts in the array
	for ghost_position in ghost_array:
		# Get the theta and phi angles for the ghost's position
		xy = ghost_position[0]**2 + ghost_position[1]**2
		ghost_r = np.sqrt(xy + ghost_position[2]**2)
		ghost_theta = np.arctan2(ghost_position[2], np.sqrt(xy)) # for elevation angle defined from Z-axis down
		ghost_phi = np.arctan2(ghost_position[1], ghost_position[0])

		relative_pitch = ghost_theta - detector_theta
		relative_yaw = detector_phi - ghost_phi

		if relative_pitch**2 + relative_yaw**2 < 0.5**2:
			detected_ghost = {}
			detected_ghost["located"] = False
			# detected_ghost["type"] = "temporary"

			detected_ghost["relative_pitch"] = relative_pitch
			detected_ghost["relative_yaw"] = relative_yaw

			if relative_pitch**2 + relative_yaw**2 < 0.05**2:
				# print("GHOST DETECTED")
				detected_ghost["type"] = "temporary"
				detected_ghost["located"] = True
				# Here would be a good place to have ghost health

			detected_ghosts.append(detected_ghost)

	return detected_ghosts


################
# Hardware setup

button_dict = {
	"trigger": 19,
	"rot_1": 16,
	"rot_2": 13,
	"knob": 6,
	"blue_1": 20,
	"blue_2": 26,
}

pixels = neopixel.NeoPixel(
	board.D21,
	50,
	brightness = 0.3,
	auto_write = False,
	pixel_order = 'GRB',
)

GPIO.setwarnings(False) # Ignore warning for now
for button in button_dict.keys():
	GPIO.setup(button_dict[button], GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Input; pulled low (off)

pygame.mixer.pre_init()
pygame.mixer.init()

pixels.fill((0, 0, 0))
pixels.show()


#################
# Begin main loop

# Initialize ghosts
num_ghosts = 1
# ghost_array = populate_ghost_array(num_ghosts)
ghost_array = []

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
	s.connect((HOST, PORT))

	# Get initial quaternion from the G.P.U.
	s.sendall(b'\x01\x02\xDE\xAD')
	data = s.recv(64)
	dt = np.dtype(float)
	dt = dt.newbyteorder('>')
	initial_quat = np.frombuffer(data, dtype=dt, count=4)

	# Get orientation matrix from the initial quaternion
	quat_rot_matrix = quaternion_rotation_matrix(initial_quat)
	adjusted_quat_rot_matrix = np.matmul(sensor_adjustment_matrix, quat_rot_matrix)
	orientation_matrix = LA.inv(adjusted_quat_rot_matrix)

	frame_counter = 0
	click_counter = 0
	ghost_scan = False
	while True:
		pixels.fill((0, 0, 0))

		#################
		# Ghost detection

		# Get latest quaternion from the G.P.U.
		s.sendall(b'\x01\x02\xDE\xAD')
		data = s.recv(64)
		dt = np.dtype(float)
		dt = dt.newbyteorder('>')
		quat = np.frombuffer(data, dtype=dt, count=4)

		detected_ghosts = detect_ghosts(
			quat, ghost_array, orientation_matrix)

		if len(detected_ghosts)>0:
			pixels.fill((0, 0, 0))

			# print("Relative pitch =", detected_ghosts[0]["relative_pitch"])
			# print("Relative yaw   =", detected_ghosts[0]["relative_yaw"])

			ghost_angle = np.arctan2(detected_ghosts[0]["relative_pitch"], detected_ghosts[0]["relative_yaw"])
			# ghost_angle += np.pi + np.pi

			# print(ghost_angle)

			center_led = int(ghost_angle/(2*np.pi)*25.0)
			pixels[center_led] = (255, 0, 255)

			click_counter += 1

			if (detected_ghosts[0]["relative_pitch"]**2 + detected_ghosts[0]["relative_yaw"]**2) < 0.4**2:
				pixels[(center_led+1)%25] = (255, 0, 255)
				pixels[(center_led-1)%25] = (255, 0, 255)
				click_counter += 1

			if (detected_ghosts[0]["relative_pitch"]**2 + detected_ghosts[0]["relative_yaw"]**2) < 0.3**2:
				pixels[(center_led+2)%25] = (255, 0, 255)
				pixels[(center_led-2)%25] = (255, 0, 255)
				click_counter += 1

			if (detected_ghosts[0]["relative_pitch"]**2 + detected_ghosts[0]["relative_yaw"]**2) < 0.2**2:
				pixels[(center_led+3)%25] = (255, 0, 255)
				pixels[(center_led-3)%25] = (255, 0, 255)
				click_counter += 2

			if (detected_ghosts[0]["relative_pitch"]**2 + detected_ghosts[0]["relative_yaw"]**2) < 0.1**2:
				pixels[(center_led+4)%25] = (255, 0, 255)
				pixels[(center_led-4)%25] = (255, 0, 255)
				click_counter += 2

			if (detected_ghosts[0]["relative_pitch"]**2 + detected_ghosts[0]["relative_yaw"]**2) < 0.05**2:
				pixels[(center_led+5)%25] = (255, 0, 255)
				pixels[(center_led-5)%25] = (255, 0, 255)
				click_counter += 2

			if detected_ghosts[0]["located"]:
				if (frame_counter/2)%2 == 0:
					pixels.fill((0, 255, 0))
				else:
					pixels.fill((0, 0, 0))
				click_counter += 10

		#########
		# Buttons

		# Trigger action
		if GPIO.input(button_dict["trigger"]) == GPIO.HIGH:
			if len(detected_ghosts)>0 and detected_ghosts[0]["located"]:
				pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/ghost_capture.wav'), maxtime=1000)
				ghost_array.pop(0) # Bit of a hack; we're always detecting the 0th ghost
				print("Ghost captured!")
				click_counter = 0
				pixels.fill((255, 0, 0))
				pixels.show()
				time.sleep(1.5)
				pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/capture_success.wav'), maxtime=1500)
				time.sleep(1.5)
				pixels.fill((0, 0, 0))
				pixels.show()
			else:
				if not ghost_scan:
					print("Begin scanning for ghosts")
					# some kind of audio feedback for starting the scan
					# Here is where you send the signal to the GPU to start reading ADC data
				ghost_scan = True
		else:
			if ghost_scan:
				print("Stopping scan")
				# Here is where you send the signal to the GPU to stop reading ADC data
				test_value = np.random.uniform(0.0, 100.)
				if test_value > 67:
					ghost_array = add_ghost_to_array(ghost_array)
					pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/ghost_in_area_2.wav'), maxtime=3736)
					pixels.fill((0, 0, 0))
					pixels.show()
					time.sleep(0.05)
					for _ in range(4):
						pixels.fill((0,255, 0))
						pixels.show()
						time.sleep(0.5)
						pixels.fill((0, 0, 0))
						pixels.show()
						time.sleep(0.44)
					pixels.fill((0, 0, 0))
					pixels.show()
				elif test_value > 33:
					# pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/scan_pass.wav'), maxtime=1505)
					pixels.fill((255, 255, 0))
					pixels.show()
					time.sleep(1.2)
					pixels.fill((0, 0, 0))
					pixels.show()
					time.sleep(0.3)
				else:
					pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/scan_pass.wav'), maxtime=1505)
					pixels.fill((255, 0, 0))
					pixels.show()
					time.sleep(1.2)
					pixels.fill((0, 0, 0))
					pixels.show()
					time.sleep(0.3)
			ghost_scan = False

		# Field reorientation
		if GPIO.input(button_dict["blue_2"]) == GPIO.HIGH:
			print("Recalibrating orientation")

			pixels.fill((0, 0, 255))
			pixels.show()
			time.sleep(0.5)

			s.sendall(b'\x01\x02\xDE\xAD')
			data = s.recv(64)
			dt = np.dtype(float)
			dt = dt.newbyteorder('>')
			quat = np.frombuffer(data, dtype=dt, count=4)

			quat_rot_matrix = quaternion_rotation_matrix(quat)
			adjusted_quat_rot_matrix = np.matmul(sensor_adjustment_matrix, quat_rot_matrix)
			orientation_matrix = LA.inv(adjusted_quat_rot_matrix)

			time.sleep(0.5)
			pixels.fill((0, 0, 0))
			pixels.show()

		# Reset ghost array
		if GPIO.input(button_dict["knob"]) == GPIO.HIGH:
			print("Clearing ghost array...")
			ghost_array = []
			time.sleep(1.0)

		# Get ghost count
		if GPIO.input(button_dict["blue_1"]) == GPIO.HIGH:
			if len(ghost_array) == 5:
				pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/five.wav'), maxtime=372)
				time.sleep(0.372)
			elif len(ghost_array) == 4:
				pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/four.wav'), maxtime=409)
				time.sleep(0.409)
			elif len(ghost_array) == 3:
				pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/three.wav'), maxtime=398)
				time.sleep(0.398)
			elif len(ghost_array) == 2:
				pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/two.wav'), maxtime=325)
				time.sleep(0.325)
			elif len(ghost_array) == 1:
				pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/one.wav'), maxtime=381)
				time.sleep(0.381)
			else:
				pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/zero.wav'), maxtime=567)
				time.sleep(0.567)


		######
		# LEDs
		if ghost_scan:
			for i in range(25):
				output_pixel = [0, 0, 0]
				if (frame_counter + i)%5 == 0:
					output_pixel[0] += 255
				if (frame_counter + 5 - i)%5 == 0:
					output_pixel[2] += 255
				pixels[i] = output_pixel


		##############
		# Audio events

		if click_counter > 30:
			pygame.mixer.Channel(0).play(pygame.mixer.Sound('/home/ghost/audio/ping_short.wav'), maxtime=334)
			click_counter = 0

		# Audio feedback events
		if len(detected_ghosts)>0:
			if detected_ghosts[0]["located"]:
				print("GHOST DETECTED!")
			else:
				# print("Ghost in area!")
				pass
		else:
			#print("<weak signal>")
			pass

		time.sleep(0.1)
		pixels.show()
		frame_counter += 1
