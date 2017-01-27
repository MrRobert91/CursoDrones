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
class PID:
    """PID Controller
    """
    def __init__(self, P=0.008, I=0.0003, D=0.01, Derivator=0, Integrator=0, Integrator_max=1000, Integrator_min=-1000):
        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min


    def update(self, feedback_error_actual):
        #feedback_value es el error actual
        #Parte proporcional
        self.valueP = feedback_error_actual * self.Kp
        print("valueP: "+ str(self.valueP))
        #Parte Derivada: Se multiplica la constant Kp por la resta del
        # error anterior al actual
        #self.valueD = 0
        self.valueD = self.Kd * (feedback_error_actual - self.Derivator)
        print("Kd: "+ str(self.Kd))
        print("feedback_error_actual: "+ str(feedback_error_actual))
        print("Derivator: "+ str(self.Derivator))
        print("valueD: "+ str(self.valueD))

        self.Derivator=feedback_error_actual


        #Parte Integrsal
        # En Integrator vamos sumando el error actual, de forma que se va
        # acumulando en la variable
        self.Integrator = self.Integrator + feedback_error_actual
        print("Integrator: "+ str(self.Integrator))

        if self.Integrator>self.Integrator_max:
            self.Integrator=self.Integrator_max
        elif self.Integrator<self.Integrator_min:
            self.Integrator=self.Integrator_min


        self.valueI = self.Integrator * self.Ki
        print("valueI: "+ str(self.valueI))

        self.valuePID = self.valueP + self.valueI + self.valueD

        if self.valuePID > 1 :
            self.valuePID = 1

        if self.valuePID < -1 :
            self.valuePID = -1





        print("valuePID: "+ str(self.valuePID))
        print("----- ")
        #Parte Derivada

        return (self.valuePID)


class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

        #controladores PID
        self.pidx = PID()
        self.pidy = PID()
        self.pidz = PID()

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
        hMin=40
        hMax=80
        sMin=100
        sMax=255
        vMin=50
        vMax=255

        lower_blueGreen = np.array([hMin,sMin,vMin])
        upper_blueGreen = np.array([hMax,sMax,vMax])

        filtroBlueGreen = cv2.inRange(imgHsv, lower_blueGreen, upper_blueGreen)

        imgCopiaBW = np.copy(filtroBlueGreen)
        imgContornos, contours, hierarchy = cv2.findContours(imgCopiaBW, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if (len(contours)>0):
            # Find the index of the largest contour
            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            cnt=contours[max_index]
            x, y, w, h = cv2.boundingRect(cnt)

            imgFinal = cv2.rectangle(input_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            imgFinal = cv2.rectangle(input_image, (125, 85), (195, 155), (0, 0, 255), 1)
            #prueba coordenadas de la imagen
            #imgFinal = cv2.rectangle(input_image, (0, 0), (10, 10), (0, 255, 0), 1)
            #Parte de control
            #Centro del rectangulo del kobuki
            centroRectX= (x+x+w)/2
            centroRectY= (y+y+h)/2

            coordenadaXKobukiImg = centroRectX
            coordenadaYKobukiImg = centroRectY

            print("coordenadasKobukiImg: "+ str(coordenadaXKobukiImg)+ " "+ str(coordenadaYKobukiImg))

            #las coordenadas del drone son justo el centro de la imagen
            hd, wd = imgFinal.shape[:2]
            print("tamaño imagen h, w: "+ str(hd)+ " "+ str(wd))
            coordenadaXDroneImg = wd/2
            coordenadaYDroneImg = hd/2
            print("coordenadasDroneImg: "+ str(coordenadaXDroneImg)+ " "+ str(coordenadaYDroneImg))

            errorImgX = coordenadaXKobukiImg - coordenadaXDroneImg
            errorImgY = coordenadaYKobukiImg - coordenadaYDroneImg

            print("errorXImg: "+ str(errorImgX)+ " errorYImg: "+ str(errorImgY))
            print("----------------------------------------- ")


            #Se pasan sdirectamente los errores de la imagen
            errorX = errorImgX
            errorY = errorImgY

            print("errorX: "+ str(errorX)+ " errorY: "+ str(errorY))
            print("----------------------------------------- ")
            velx = self.pidx.update(errorX)
            vely = self.pidy.update(errorY)

            if ((errorX < (25))and(errorX > (-25)) and(errorY < (25))and(errorY > (-25))):
                print("Me estoy parando!!!")
                self.cmdvel.sendCMDVel(0,0,0,0,0,0)
            else:
                #viene mal en el guion de practicas, los valores de x e y estan cambiados
                print("Me estoy moviendo!!!")
                print("velx: "+ str(velx)+ " vely: "+ str(vely))
                #La cordenada Y de la imagen y del plano estan al reves

                self.cmdvel.sendCMDVel(-vely,-velx,0,0,0,0)


            # si es muy prqueño el kobuki bajamos
            print("Tamaño del kobuki w: " + str(w) + ", h: "+ str(h))
            if (w<15 or h<15):

                print("El Kobuki se ve muy pequeño, bajo a buscarlo")
                #To-Do
                #self.cmdvel.sendCMDVel(0,0,-1,0,0,0)


        else:
            #Si no encuentra contorno del kobuki
            print("He perdido al Kobuki, subo a buscarlo")
            #To-Do
            #self.cmdvel.sendCMDVel(0,0,0.5,0,0,0)





        if input_image is not None:
            self.camera.setColorImage(input_image)
            self.camera.setThresoldImage(filtroBlueGreen)
            '''
            If you want show a thresold image (black and white image)
            self.camera.setThresoldImage(bk_image)
            '''
