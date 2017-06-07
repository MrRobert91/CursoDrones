import threading
import time
import cv2
import numpy as np
from datetime import datetime

from parallelIce.cameraClient import CameraClient
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

        imagenCamera = self.camera.getImage()
        imagenCopia = np.copy(imagenCamera)
        blur = cv2.blur(imagenCopia,(5,5))
        #cv2.imshow('imagenCopia',imagenCopia)
        #cv2.imshow('blur',blur)

        imgHsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        #imgRGB = cv2.cvtColor(imagenCopia, cv2.COLOR_BGR2RGB)
        #cv2.imshow('imgRGB',imgRGB)
        """
        hMin=0
        hMax=10
        sMin=180
        sMax=255
        vMin=180
        vMax=255
        """
        """
        hMin=0
        hMax=10
        sMin=150
        sMax=255
        vMin=150
        vMax=255
        """
        """
        hMin=40
        hMax=80
        sMin=100
        sMax=255
        vMin=50
        vMax=255
        """


        #lower_red = np.array([hMin,sMin,vMin])
        #upper_red = np.array([hMax,sMax,vMax])

        #really a blue filter
        lower_red = np.array([110,50,50])
        upper_red = np.array([130,255,255])

        filtroRojo = cv2.inRange(imgHsv, lower_red, upper_red)
        imgCopiaBW = np.copy(filtroRojo)
        cv2.imshow('imgCopiaBW', imgCopiaBW)

        kernel = np.ones((5,5), np.uint8)
        imgDilate = cv2.dilate(imgCopiaBW, kernel, iterations=4)
        imgErosion = cv2.erode(imgDilate, kernel, iterations=4)

        cv2.imshow('imgErosion', imgErosion)

        imgContornos, contours, hierarchy = cv2.findContours(imgErosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        print("contornos:")
        print(len(contours))

        if (len(contours)>0):

            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            cnt=contours[max_index]
            x, y, w, h = cv2.boundingRect(cnt)
            imagenCopiaRGB= cv2.cvtColor(imagenCopia, cv2.COLOR_BGR2RGB)
            #imgFinal = cv2.rectangle(imagenCopiaRGB, (x, y), (x + w, y + h), (0, 255, 0), 2)

            #mouse rectangle position
            imgFinal = cv2.rectangle(imagenCopiaRGB, (x, y), (x + w, y + h), (255, 0, 0), 2)

            #image centre
            imgFinal = cv2.rectangle(imagenCopiaRGB, (125, 85), (195, 155), (0, 255, 0), 1)
            cv2.imshow('imgFinal',imgFinal)


        tmp = self.navdata.getNavdata()
        if tmp is not None:
            print ("State: " +str(tmp.state))
            print ("Altitude: " +str(tmp.altd))
            print ("Vehicle: " +str(tmp.vehicle))
            print ("Battery %: " +str(tmp.batteryPercent))
