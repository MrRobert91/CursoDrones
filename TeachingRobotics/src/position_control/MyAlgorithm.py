import threading
import time
from datetime import datetime

import math
import jderobot
from Beacon import Beacon

from parallelIce.cameraClient import CameraClient
from parallelIce.navDataClient import NavDataClient
from parallelIce.cmdvel import CMDVel
from parallelIce.extra import Extra
from parallelIce.pose3dClient import Pose3DClient

time_cycle = 80

class PID:
    """PID Controller
    """
    def __init__(self, P=0.3, I=0.001, D=0.4, Derivator=0, Integrator=0, Integrator_max=100, Integrator_min=-100):
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
        #Parte Derivada: Se multiplica la constant Kp por la resta del
        # error anterior al actual
        #self.valueD = 0
        print("valueP: "+ str(self.valueP))
        self.valueD = self.Kd * (feedback_error_actual - self.Derivator)
        self.Derivator=feedback_error_actual
        print("valueD: "+ str(self.valueD))

        #Parte Integrsal
        # En Integrator vamos sumando el error actual, de forma que se va
        # acumulando en la variable
        self.Integrator = self.Integrator + feedback_error_actual

        #print("Integrator: "+ str(self.Integrator))


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



        print("-------------------Fin PID---------------------------")
        #Parte Derivada

        return (self.valuePID)


        """
        if ((feedback_value * self.Kp) > 1):
            return 1
        elif  ((feedback_value * self.Kp) < (-1)):
            return -1
        else:
            return (feedback_value * self.Kp)
        """
    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

class MyAlgorithm(threading.Thread):

    def __init__(self, camera, navdata, pose, cmdvel, extra):
        self.camera = camera
        self.navdata = navdata
        self.pose = pose
        self.cmdvel = cmdvel
        self.extra = extra

        self.beacons=[]
        self.initBeacons()
        self.minError=0.01

        self.pidx = PID()
        self.pidy = PID()

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)

    def initBeacons(self):
        self.beacons.append(Beacon('baliza1',jderobot.Pose3DData(0,5,0,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('baliza2',jderobot.Pose3DData(5,0,0,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('baliza3',jderobot.Pose3DData(0,-5,0,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('baliza4',jderobot.Pose3DData(-5,0,0,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('baliza5',jderobot.Pose3DData(10,0,0,0,0,0,0,0),False,False))
        self.beacons.append(Beacon('inicio',jderobot.Pose3DData(0,0,0,0,0,0,0,0),False,False))

    def getNextBeacon(self):
        for beacon in self.beacons:
            if beacon.isReached() == False:
                return beacon

        return None

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

        balizaActual = self.getNextBeacon()
        if (balizaActual != None):
            coordenadaXBaliza1 = balizaActual.getPose().x
            coordenadaYBaliza1 = balizaActual.getPose().y
            balizaActual.setActive(True)
        else:
            print("Final del viaje")
            #Cuando ya no queden balizas irá a la posicion 0, 0
            coordenadaXBaliza1 = 0
            coordenadaYBaliza1 = 0



        coordenadaXDrone = self.pose.getPose3D().x
        coordenadaYDrone = self.pose.getPose3D().y
        print("balizaXY: "+ str(coordenadaXBaliza1)+ " "+ str(coordenadaYBaliza1))
        print("droneXY: "+ str(coordenadaXDrone)+" "+str(coordenadaYDrone))

        errorX = coordenadaXBaliza1 - coordenadaXDrone
        errorY = coordenadaYBaliza1 - coordenadaYDrone
        print("errorX: "+ str(errorX)+" errorY: "+ str(errorY))

        velx = self.pidx.update(errorX)
        vely = self.pidy.update(errorY)

        print("velX: "+str(velx)+ " velY: "+str(vely))

    #    pidy = PID()
    #    pidx = PID()
        if ((errorX < (0.1))and(errorX > (-0.1)) and(errorY < (0.1))and(errorY > (-0.1))):
            print("Me estoy parando!!!")
            if balizaActual != None :
                balizaActual.setActive(False)
                balizaActual.setReached(True)
                #self.cmdvel.sendCMDVel(0,0,0,0,0,0)
            else:
                self.cmdvel.sendCMDVel(0,0,0,0,0,0)



        else:
            #viene mal en el guion de practicas, los valores de x e y estan cambiados
            print("Me estoy moviendo!!!")
            self.cmdvel.sendCMDVel(velx,vely,0,0,0,0)

        print("----------------------------------------")



        pass
