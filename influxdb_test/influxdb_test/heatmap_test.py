'''
#***************original heat map***************** can run
import cv2
import numpy as np

# Create a VideoCapture object for the D435 camera
#the first usb port on the front of the chassis
cap = cv2.VideoCapture(4)

# Define the size of the motion heatmap
heatmap_size = (640, 480)

# Create a motion heatmap array
heatmap = np.zeros(heatmap_size, dtype=np.float32)

# Define decay factor and threshold value
decay_factor = 0.05
threshold_value = 10

# Initialize previous frame
prev_gray = None

# Process live video frames
while True:
	# Get a frame from the camera
	ret, frame = cap.read()
	
	# Resize the frame to the heatmap size
	frame = cv2.resize(frame, heatmap_size)
	
	# Convert the frame to grayscale
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	# If this is the first frame, set the previous frame to the current frame
	if prev_gray is None:
		prev_gray = gray
		
		continue
	

	# Calculate the absolute difference between the current frame and the previous frame
	diff = cv2.absdiff(gray, prev_gray)
	
	# Apply a threshold to the difference image
	threshold = 25
	_, thresh = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
	
	# Blur the thresholded image
	kernel_size = (5, 5)
	blur = cv2.GaussianBlur(thresh, kernel_size, 0)
	
	# Create a binary image of the motion areas
	_, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY)
	
	# Transpose the binary array to match the shape of the heatmap array
	binary = np.transpose(binary)
	
	# Update the motion heatmap
	heatmap += binary
	heatmap *= (1 - decay_factor) # apply decay factor to reduce heatmap intensity over time
	
	# Threshold the heatmap to remove noise and only show active regions
	_, heatmap_thresh = cv2.threshold(heatmap, threshold_value, 255, cv2.THRESH_BINARY)

	# Apply a color map to the heatmap
	heatmap_color = cv2.applyColorMap(np.uint8(heatmap * 10), cv2.COLORMAP_HOT)
	
	# Rotate the heatmap 90 degrees clockwise
	heatmap_color = cv2.rotate(heatmap_color, cv2.ROTATE_90_CLOCKWISE)
	
	# Mirror the heatmap horizontally
	heatmap_color = cv2.flip(heatmap_color, 1)

	# Display the video and the motion heatmap
	cv2.imshow('Video', frame)
	cv2.imshow('Heatmap', heatmap_color)
	
	# Set the previous frame to the current frame
	prev_gray = gray
	
	# Exit the loop if 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

# Release the camera and destroy all windows
cap.release()
cv2.destroyAllWindows()
'''





'''
# **************heat map of a moving point (heatmap+nuitrack)*************************
from PyNuitrack import py_nuitrack
import cv2
import numpy as np


# Define the radius of the circular mask around the moving point
mask_radius = 30  # example radius, replace with your desired radius

# nuitrack setup part
nuitrack = py_nuitrack.Nuitrack()
nuitrack.init()
nuitrack.create_modules()
nuitrack.run()


# Create a VideoCapture object for the D435 camera
# the first usb port on the front of the chassis
#cap = cv2.VideoCapture(4) #### *************************Problem is here *********************************
heatmap_size = (640, 480)
heatmap = np.zeros(heatmap_size, dtype=np.float32)

decay_factor = 0.1

prev_gray = None

# Process live video frames
while True:
	key = cv2.waitKey(1)
	nuitrack.update()
	data = nuitrack.get_skeleton()
	data_instance=nuitrack.get_instance()
	img_depth = nuitrack.get_depth_data()
	
	#**********if img_depth.size:
	# nuitrack part
	cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
	img_depth = np.array(cv2.cvtColor(img_depth,cv2.COLOR_GRAY2RGB), dtype=np.uint8)
	img_color = nuitrack.get_color_data()
	
	right_x=0
	right_y=0
	for skeleton in data.skeletons:
		#turn unit to meter
		right_x = skeleton.right_hand.projection[0]
		right_y = skeleton.right_hand.projection[1]
		print("right hand x: %.4f, y: %.4f"%(right_x,right_y))

	#cv2.imshow('Image', img_depth)
	cv2.imshow('Color', img_color)

	# heat map part
	#frame = img_depth
	frame = img_color
	frame = cv2.resize(frame, heatmap_size)
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	
	if prev_gray is None:
		prev_gray = gray
		
		continue
	
	diff = cv2.absdiff(gray, prev_gray)
	
	threshold = 25
	_, thresh = cv2.threshold(diff, threshold, 255, cv2.THRESH_BINARY)
	
	kernel_size = (5, 5)
	blur = cv2.GaussianBlur(thresh, kernel_size, 0)
	_, binary = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY)
	binary = np.transpose(binary)

	heatmap += binary
	#heatmap *= (1 - decay_factor) # apply decay factor to reduce heatmap intensity over time

	# Create a circular binary mask around the moving point
	moving_point = (int(right_y), int(right_x))
	mask = np.zeros(heatmap_size, dtype=np.uint8)
	cv2.circle(mask, moving_point, mask_radius, 1, -1)
	heatmap_filtered = heatmap * mask

	# Threshold the heatmap to remove noise and only show active regions
	#threshold_value = 10
	#_, heatmap_thresh = cv2.threshold(heatmap_filtered, threshold_value, 255, cv2.THRESH_BINARY)
	heatmap_color = cv2.applyColorMap(np.uint8(heatmap_filtered * 10), cv2.COLORMAP_HOT)
	
	# Rotate and mirror
	heatmap_color = cv2.rotate(heatmap_color, cv2.ROTATE_90_CLOCKWISE)
	heatmap_color = cv2.flip(heatmap_color, 1)

	# New add****************test****************************

	heatmap_color2 = cv2.applyColorMap(np.uint8(heatmap * 10), cv2.COLORMAP_HOT)
	heatmap_color2 = cv2.rotate(heatmap_color2, cv2.ROTATE_90_CLOCKWISE)
	heatmap_color2 = cv2.flip(heatmap_color2, 1)
	
	fin = cv2.addWeighted(heatmap_color, 0.5, heatmap_color2, 0.5, 0)
	#cv2.imshow('Heatmap', fin)
	# *******************************************************

	# Display the video and the motion heatmap
	#cv2.imshow('Video', frame)
	cv2.imshow('Heatmap', heatmap_color)
	
	
	# Set the previous frame to the current frame
	prev_gray = gray
	
	# Exit the loop if 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
	
	
nuitrack.release()
'''



'''
# ************nuitrack+heatmap every 300 frame (without 'frame = cv2.resize(frame, heatmap_size_new)' version)************** can run
from PyNuitrack import py_nuitrack
import cv2
import numpy as np

# nuitrack setup part
nuitrack = py_nuitrack.Nuitrack()
nuitrack.init()
nuitrack.create_modules()
nuitrack.run()

# Create a VideoCapture object for the D435 camera
heatmap_size = (640, 480)
#heatmap_size_new = (1440, 1080)


# cv2.resize will can the resolution of frames, so here add ratio to fix this problem
# Calculate the ratio of new and old resolutions
#ratio = np.array(heatmap_size_new) / np.array(heatmap_size_old)

point_list = []

# Process live video frames
while True:
	key = cv2.waitKey(1)
	nuitrack.update()
	data = nuitrack.get_skeleton()
	data_instance=nuitrack.get_instance()
	img_depth = nuitrack.get_depth_data()

	# nuitrack part
	cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
	img_depth = np.array(cv2.cvtColor(img_depth,cv2.COLOR_GRAY2RGB), dtype=np.uint8)
	img_color = nuitrack.get_color_data()

	right_x=0
	right_y=0
	for skeleton in data.skeletons:
		#get right hand projection coordination
		right_x = skeleton.right_hand.projection[0]
		right_y = skeleton.right_hand.projection[1]
		print("right hand x: %.4f, y: %.4f"%(right_x,right_y))
		point_list.append([int(right_x), int(right_y)])

	# heat map part
	#frame = img_depth
	frame = img_color
	frame = cv2.resize(frame, heatmap_size)

	cv2.imshow('Video', frame)
	
	# Check if we have enough points to draw the heatmap
	if len(point_list) >= 300:
		# Create the heatmap from the point list
		heatmap_points = np.zeros(heatmap_size, dtype=np.float32)
		for point in point_list:
			#x, y = point
			x = point[0]
			y = point[1]
			if x > heatmap_size[0] or x < 0 or y > heatmap_size[1] or y < 0:
				x = 0
				y = 0
			heatmap_points[int(x), int(y)] += 1
		
		heatmap_points = cv2.GaussianBlur(heatmap_points, (45, 45), 0)
		heatmap_points = cv2.normalize(heatmap_points, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
		heatmap_color = cv2.applyColorMap(np.uint8(heatmap_points * 1), cv2.COLORMAP_JET)

		# Rotate and mirror
		heatmap_color = cv2.rotate(heatmap_color, cv2.ROTATE_90_CLOCKWISE)
		heatmap_color = cv2.flip(heatmap_color, 1)

		result = cv2.addWeighted(frame, 0.5, heatmap_color, 0.5, 0)

		# Display the heatmap
		#cv2.imshow('Heatmap', heatmap_color)
		cv2.imshow('result', result)

		# Reset the point list
		point_list = []
	
	# Exit the loop if 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

nuitrack.release()
'''

# ************nuitrack+heatmap every 300 frame + InfluxDB (without 'frame = cv2.resize(frame, heatmap_size_new)' version)************** can run
from PyNuitrack import py_nuitrack
import cv2
import numpy as np
from influxdb_client import InfluxDBClient, Point
import time

# nuitrack setup part
nuitrack = py_nuitrack.Nuitrack()
nuitrack.init()
nuitrack.create_modules()
nuitrack.run()

# Create a VideoCapture object for the D435 camera
heatmap_size = (640, 480)
#heatmap_size_new = (1440, 1080)


point_list = []

#set necessary info to connect InfluxDBClient
influxdb_client = InfluxDBClient(url="http://localhost:8086", \
								token="EZZyYWflA8jgJFT1J5TfTkTbgECQQzcIbEXvTDKwBVKntwRm4JyAEy3wzjzJE20i-i-8k9vFbIO1WDxsGNQSPw==", \
								org="PointCloud")
influxdb_write_api = influxdb_client.write_api()

# draw skeleton
def draw_skeleton(image):
		point_color = (59, 164, 0)
		for skel in data.skeletons:
			for el in skel[1:]:
				x = (round(el.projection[0]), round(el.projection[1]))
				cv2.circle(image, x, 8, point_color, -1)

# Process live video frames
while True:
	key = cv2.waitKey(1)
	nuitrack.update()
	data = nuitrack.get_skeleton()
	data_instance=nuitrack.get_instance()
	img_depth = nuitrack.get_depth_data()

	# nuitrack part
	cv2.normalize(img_depth, img_depth, 0, 255, cv2.NORM_MINMAX)
	img_depth = np.array(cv2.cvtColor(img_depth,cv2.COLOR_GRAY2RGB), dtype=np.uint8)
	img_color = nuitrack.get_color_data()

	right_x=0
	right_y=0
	for skeleton in data.skeletons:
		#get right hand projection coordination
		right_x = skeleton.right_hand.projection[0]
		right_y = skeleton.right_hand.projection[1]
		print("right hand x: %.4f, y: %.4f"%(right_x,right_y))
		point_list.append([int(right_x), int(right_y)])

		#get right hand xyd position, turn to meter
		right_x=(skeleton.right_hand.real[0])*0.001
		right_y=(-skeleton.right_hand.real[1])*0.001
		#here nuitrack coordinate system (y) is different with D435 camera in pointcloud
		right_d=(skeleton.right_hand.real[2])*0.001
	
		#print("right hand x: %.4f, y: %.4f, d: %.4f"%(right_x,right_y,right_d))
		timestamp = int(time.time()*1000)
		data_point = Point("right_hand_position") \
            		.field("x", right_x) \
			    	.field("y", right_y) \
				    .field("d", right_d) \
            		.time(timestamp,"ms")
		influxdb_write_api.write(bucket="min_distance_test", record=data_point)
		

	# heat map part
	#frame = img_depth
	frame = img_color
	frame = cv2.resize(frame, heatmap_size)

	# draw skeleton
	draw_skeleton(img_depth)

	cv2.imshow('Video', img_depth)
	
	# Check if we have enough points to draw the heatmap
	if len(point_list) >= 300:
		# Create the heatmap from the point list
		heatmap_points = np.zeros(heatmap_size, dtype=np.float32)
		for point in point_list:
			#x, y = point
			x = point[0]
			y = point[1]
			if x >= heatmap_size[0] or x < 0 or y >= heatmap_size[1] or y < 0:
				x = 0
				y = 0
			heatmap_points[int(x), int(y)] += 1
		
		heatmap_points = cv2.GaussianBlur(heatmap_points, (45, 45), 0)
		heatmap_points = cv2.normalize(heatmap_points, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
		heatmap_color = cv2.applyColorMap(np.uint8(heatmap_points * 1), cv2.COLORMAP_JET)

		# Rotate and mirror
		heatmap_color = cv2.rotate(heatmap_color, cv2.ROTATE_90_CLOCKWISE)
		heatmap_color = cv2.flip(heatmap_color, 1)

		result = cv2.addWeighted(frame, 0.5, heatmap_color, 0.5, 0)

		# Display the heatmap
		#cv2.imshow('Heatmap', heatmap_color)
		cv2.imshow('result', result)
		cv2.imshow('Heatmap',heatmap_color)

		# Reset the point list
		point_list = []
	
	# Exit the loop if 'q' is pressed
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break

nuitrack.release()

