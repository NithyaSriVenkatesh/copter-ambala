#!/usr/bin/env python
'''
An example program to capture cuav-compliant images using OpenCV.
NOTE: The capture time of the images (represented in the filename) must be <100ms
precise, otherwise the geolocation will be inaccurate

This uses Python 2.7/3.x and OpenCV2

Use "sudo apt-get install opencv-python" to install OpenCV2
'''
import time, cv2, os, threading, math, csv
import numpy as np
from cuav.image import scanner	
from cuav.lib import cuav_util
from dronekit import connect
import Footprint_MIT_1 as FP
from pymavlink import mavutil
from sklearn.cluster import DBSCAN
# from functions import find_wifi_address
# import RPi.GPIO as GPIO 



master = mavutil.mavlink_connection("192.168.6.152:14560",baud=115200)
print('master',master)	
lat, lng, rel_alt, yaw_angle, roll, pitch= 0,0,0,0,0,0

current_path = os.getcwd()
print(current_path)
Images={}

rawFileNumber, detectFileNumber = 1, 1

flag = 0
flag1 = 0
Flash_flag = 1

with open(current_path + "/Flag.csv", "w") as f:
    csvwriter = csv.writer(f)
    csvwriter.writerow([0])

with open(current_path + "/Flag1.csv", "w") as f:
    csvwriter = csv.writer(f)
    csvwriter.writerow([0])
    
with open(current_path + "/RawFileNumber.csv", "r") as f:
    csvreader = csv.reader(f)
    for row in csvreader:
        rawFileNumber = int(row[0])
    
with open(current_path + "/DetectFileNumber.csv", "r") as f:
    csvreader = csv.reader(f)
    for row in csvreader:
        detectFileNumber = int(row[0])
    
    
frames_path = current_path + "/frames_output_" + str(rawFileNumber)

detect_path = current_path + "/detect_" + str(detectFileNumber)


def vehicle_location():
    global master, lat, lng, rel_alt, yaw_angle, roll, pitch, flag, flag1
    # if !initial1:
    while True:
        if flag and flag1:
            break
        msg = master.recv_match()
        if not msg:
            lat = lat
            lng = lng
            rel_alt = rel_alt
            yaw_angle = yaw_angle
            continue
        if msg.get_type() == "GLOBAL_POSITION_INT":
            lat = master.field("GLOBAL_POSITION_INT", "lat", 0) * 1.0e-7
            lng = master.field("GLOBAL_POSITION_INT", "lon", 0) * 1.0e-7
            rel_alt = master.field("GLOBAL_POSITION_INT", "relative_alt", 0) * 1.0e-3
        if msg.get_type() == "ATTITUDE":
            yaw = master.field("ATTITUDE", "yaw", 0)
            pitch = master.field("ATTITUDE", "pitch", 0)
            roll = master.field("ATTITUDE", "roll", 0)
            pitch_deg = math.degrees(pitch)
            roll_deg = math.degrees(roll)
            yaw_1 = math.degrees(yaw)
            yaw_2 = 360 + yaw_1
            if yaw_2 > 360:
                yaw_angle = yaw_2 - 360
            else:
                yaw_angle = yaw_2
            lat = lat
            lng = lng
            roll = roll_deg
            pitch = pitch_deg
            yaw = yaw_angle
            rel_alt = rel_alt
        #print("!!!!!!!!",lat, lng, rel_alt, yaw_angle, roll, pitch)

def frame_time(t):
    '''return a time string for a filename with 0.01 sec resolution'''
    # round to the nearest 100th of a second
    t += 0.005
    hundredths = int(t * 100.0) % 100
    return "%s%02uZ" % (time.strftime("%Y%m%d%H%M%S", time.gmtime(t)), hundredths)

def Capture_Images():
    global lat, lng, rel_alt, yaw_angle, roll, pitch, current_path, Images, flag, frames_path, detect_path, rawFileNumber, detectFileNumber, Flash_flag
    #Open the default camera on the system
    #camera = cv2.VideoCapture("rtspsrc location=rtsp://192.168.6.124:8554/main.264 latency=0 ! decodebin ! videoconvert ! appsink",cv2.CAP_GSTREAMER)
    camera = cv2.VideoCapture("rtspsrc location=rtsp://192.168.6.122:8554/main.264 latency=0 ! decodebin ! videoconvert ! appsink", cv2.CAP_GSTREAMER)

    
    
    with open(current_path + "/RawFileNumber.csv", "r") as f:
        csvreader = csv.reader(f)
        for row in csvreader:
            rawFileNumber = int(row[0])
        
    with open(current_path + "/DetectFileNumber.csv", "r") as f:
        csvreader = csv.reader(f)
        for row in csvreader:
            detectFileNumber = int(row[0])
        
        
    frames_path = current_path + "/frames_output_" + str(rawFileNumber)

    detect_path = current_path + "/detect_" + str(detectFileNumber)

    #camera.grab()

    index1=0
    previous_time = time.time()
    try:
        while True:
            #grab the image and get timestamp
            frametime = frame_time(time.time())
            with open(current_path + "/Flag.csv", "r") as f:
                csvreader = csv.reader(f)
                for row in csvreader:
                    flag = int(row[0])
            
            #camera.grab()
            current_time = time.time()
            if current_time - previous_time >= 0.5:
                #Get the grabbed image from the device and write to file as png
                return_value, image = camera.read()

                if return_value:
                    target_path = os.path.join(frames_path+ "/UAV1_0_{}_{}_{}_{}_{}_{}_{}.jpg".format(index1, lat, lng, rel_alt, yaw_angle, roll, pitch))
                    cv2.imwrite(target_path,image)
                    Images[index1] = str(target_path)
                    index1+=1
                #cv2.imshow("frame",image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                if flag:
                    break
                previous_time = current_time
                Flash_flag = 0
    except KeyboardInterrupt:
        pass

    #Finally, shutdown the camera
    print("Closing camera")
    del(camera)

def FeatureExtraction():
    global Images,flag,flag1,featureDetectNumber,feature_detect_path,detect_path
    Ls,Hs,focal_length=(9,6,4.7)
    rad1=np.pi/180
    index1=0        
    j=0
    tar_count1=0
    
    with open(current_path + "/FeatureDetectNumber.csv", "r") as f:
        csvreader = csv.reader(f)
        for row in csvreader:
            featureDetectNumber = int(row[0])
            
    feature_detect_path = current_path + "/feature_detect_" + str(featureDetectNumber)
    print("feature_detect_path",feature_detect_path)
        
    while True:
        if j not in Images:
            if j >= len(Images) and flag:
                with open(current_path + "/Flag1.csv", "w") as f:
                    csvwriter = csv.writer(f)
                    csvwriter.writerow([2])
                    flag1=1
                    break
            continue
        try:
            cap_done = Images[j]
            image = cv2.imread(cap_done)
                
            # Convert image to grayscale for feature extraction
            gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            '''
            # Initialize ORB detector
            orb = cv2.ORB_create(nfeatures=150, edgeThreshold=15, scaleFactor=1.2, nlevels=8)

            # Detect keypoints and compute descriptors
            keypoints, descriptors = orb.detectAndCompute(gray_image, None)
            '''
            akaze = cv2.AKAZE_create(
                threshold=0.003,          # Lower threshold for more keypoints
                nOctaves=6,               # More octaves to detect features at more scales
                nOctaveLayers=4,          # Standard layer setup per octave
                diffusivity=cv2.KAZE_DIFF_WEICKERT,  # Use Weickert diffusivity
                descriptor_type=cv2.AKAZE_DESCRIPTOR_MLDB,  # Use MLDB descriptor
                descriptor_size=64        # Standard descriptor size
            )

            keypoints, descriptors = akaze.detectAndCompute(gray_image, None)

            j+=1
            if keypoints:
                keypoint_coords = np.array([kp.pt for kp in keypoints])
                clustering = DBSCAN(eps=20, min_samples=7, metric='euclidean', algorithm='auto').fit_predict(keypoint_coords) # You can tune eps and min_samples
                crop_count = 0  # Counter for naming cropped images
                unique_labels = set(clustering)

                for label in unique_labels:
                    if label == -1:
                        continue  # Skip noise points
                    
                    cluster_points = keypoint_coords[clustering == label]
                    x_min, y_min = np.min(cluster_points, axis=0)
                    x_max, y_max = np.max(cluster_points, axis=0)
                    x_min, y_min = max(0, x_min - 40), max(0, y_min - 40)  # Ensure the box doesn't go out of bounds
                    x_max, y_max = min(image.shape[1], x_max + 40), min(image.shape[0], y_max + 40)
                    crop=image[int(y_min):int(y_max), int(x_min):int(x_max)]
                    x_1=((int(x_min)*1.00+int(x_max))*0.5)
                    y_1=((int(y_min)*1.00+int(y_max))*0.5)
                    target_total = cap_done.split(".jpg")[0].split("/")[-1]
                    ar_tar = target_total.split("_")
                    #focal_length= float(get_focal_length())
                    focal_length = 1
                    (
                        tar_lat,
                        tar_lon
                    ) = FP.getPos(
                        float(ar_tar[3]),
                        float(ar_tar[4]),
                        float(ar_tar[5]),
                        float(ar_tar[6]),
                        Ls,
                        Hs,
                        focal_length,
                        float(0.00),
                        float(ar_tar[7]),
                        x_1,
                        y_1,
                        rad1,
                        1920,
                        1080,
                    )
                    tar_count1 = tar_count1 + 1
                    target_total_1 = target_total + "_" + str(crop_count) + "_" + str(x_1) + "_" + str(y_1) + "_" + str(tar_lat) + "_" + str(tar_lon) 
                    cropped_file_path = detect_path + "/" + str(target_total_1) + ".jpg"                    
                    index1+=1
                    crop_count+=1
                    try:
                        cv2.imwrite(cropped_file_path, crop)
                    except Exception as e:
                        print("Errorrrrrrrrrrrrrrr",e)
                        pass
                #bounding box
                cv2.rectangle(image,(int(x_min),int(y_min)),(int(x_max),int(y_max)),(0,255,0),2)
                target_total_1 = target_total + "_" + "bb" + "_" + str(tar_lat) + "_" + str(tar_lon)
                bb_img_path = feature_detect_path + "/" + str(target_total_1) + "_bb.jpg"

                #bb_img_path=detect_path + "/" + str(target_total_1) + ".jpg"
                try:
                	cv2.imwrite(bb_img_path,image)
                except:
                	print("BoundingBox error")
                	pass                	
        except Exception as e:
            print("Error",e)
            pass


threading.Thread(target=Capture_Images).start()
threading.Thread(target=vehicle_location).start()
threading.Thread(target=FeatureExtraction).start()
    
Ls,Hs,f=(34.6,24.9,21)
rad1=np.pi/180
index=0

    
i=0
tar_count=0

while True:
    if i not in Images:
        if i >= len(Images) and flag:
            with open(current_path + "/Flag1.csv", "w") as f:
                csvwriter = csv.writer(f)
                csvwriter.writerow([1])
                flag1=1
        continue
    cap_done = Images[i]
    print('cap_done',cap_done)
    img = cv2.imread(cap_done)
    img = np.ascontiguousarray(img)
    regions = scanner.scan(img)


    
    (wview, hview) = cuav_util.image_shape(img)
    i+=1
    for count, r in enumerate(regions):
        (x1, y1, x2, y2, score) = r
        x_1 = (x1 + x2) * 0.5
        y_1 = (y1 + y2) * 0.5
        crop_x1 = max(0, x1 - 50)
        crop_y1 = max(0, y1 - 50)
        crop_x2 = min(wview, x2 + 50)
        crop_y2 = min(hview, y2 + 50)
        crop = img[crop_y1:crop_y2, crop_x1:crop_x2]
        target_total = cap_done.split(".jpg")[0].split("/")[-1]
        ar_tar = target_total.split("_")
        
        (
            tar_lat,
            tar_lon
        ) = FP.getPos(
            float(ar_tar[3]),
            float(ar_tar[4]),
            float(ar_tar[5]),
            float(ar_tar[6]),
            Ls,
            Hs,
            f,
            float(ar_tar[8]),
            float(ar_tar[7]),
            x_1,
            y_1,
            rad1,
            1920,
            1080,
        )
        tar_count = tar_count + 1
        target_total_1 = target_total + "_" + str(tar_lat) + "_" + str(tar_lon)
        #cropped_file_path = detect_path + "/" + str(target_total_1) + ".jpg"
        index+=1
        #print(cropped_file_path)
        #try:
            #cv2.imwrite(cropped_file_path, crop)
        #except Exception as e:
            #print("Errorrrrrrrrrrrrrrr",e)
            #pass

