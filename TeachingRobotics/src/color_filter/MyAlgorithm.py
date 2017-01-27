import threading
import time
from datetime import datetime
import cv2
import numpy as np

from sensors.cameraFilter import CameraFilter
from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient


time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)


    def run (self):

        self.stop_event.clear()

        while (not self.kill_event.is_set()):

            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()

    def execute(self):
       # Add your code here

        input_image = self.camera.getImage()
        imgCopia = np.copy(input_image)
        imgBlur = cv2.GaussianBlur(imgCopia, (5, 5), 0)


        imgHsv = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2HSV)


        hMin=101
        hMax=179
        sMin=220
        sMax=255
        vMin=126
        vMax=255

        lower_red = np.array([hMin,sMin,vMin])
        upper_red = np.array([hMax,sMax,vMax])

        filtroRojo = cv2.inRange(imgHsv, lower_red, upper_red)

        imgCopiaBW = np.copy(filtroRojo)

        kernel = np.ones((5,5), np.uint8)
        imgDilate = cv2.dilate(imgCopiaBW, kernel, iterations=4)
        imgErosion = cv2.erode(imgDilate, kernel, iterations=4)



        imgContornos, contours, hierarchy = cv2.findContours(imgErosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours)>0):

            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            cnt=contours[max_index]
            x, y, w, h = cv2.boundingRect(cnt)
            imgFinal = cv2.rectangle(input_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        


        if input_image is not None:
            self.camera.setColorImage(input_image)
            self.camera.setThresoldImage(filtroRojo)

            '''
            If you want show a thresold image (black and white image)
            self.camera.setThresoldImage(bk_image)
            '''
