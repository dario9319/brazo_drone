# -*- coding: utf-8 -*-
"""
Created on Mon Jan 24 09:52:32 2022
"""



import os
from signal import signal

import rospy
from rospkg import RosPack
import sys
import serial.tools.list_ports
# Interfaz Grafica ######################################################
import python_qt_binding.QtCore as QtCore
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import *
from python_qt_binding.QtWidgets import *

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import *

from PyQt5 import QtTest


from pygame import mixer  # Load the popular external library

from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

import psutil
import signal

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from cvzone.HandTrackingModule import HandDetector

import time
from sensor_msgs.msg import JointState
from std_msgs.msg import String


from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *


# PARA CONTROLAR DYNAMIXEL SERVOS******************
# *************************************************

# Control table address
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_OPERATING_MODE     = 3

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
 
# Default setting               # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = ''     # Check which port is being used on your controller
                                                 # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque







detector = HandDetector(detectionCon=0.8, maxHands=2)


datoEslabon1 = 0.0
datoEslabon2 = 0.0
datoEslabon3 = 0.0
datoEslabon4 = 0.0
datoPinza = 0.0


rp = RosPack()
rutaSonido2 = os.path.join(rp.get_path('brazo_drone'), 'src/guide/sonidos', 'cursor-move.mp3')
rutaSonido1 = os.path.join(rp.get_path('brazo_drone'), 'src/guide/sonidos', 'click.mp3')


process_name1 = "rviz"
process_name2 = "roslaunch"
#process_name3 = "rosmaster"



pid = None


for proc in psutil.process_iter():
    
    #print(proc.name())
    if process_name1 in proc.name():
        pid1 = proc.pid
        #print(os.getpid())
      #  print(pid1)
        os.kill(pid1,signal.SIGINT)
        os.kill(pid1,signal.SIGTERM)

    if process_name2 in proc.name():
     
        pid2 = proc.pid
        #print(os.getpid())
        print(pid2)
        os.kill(pid2,signal.SIGINT)
        os.kill(pid2,signal.SIGTERM)

    
    #if process_name3 in proc.name():
     
     #   pid2 = proc.pid
        #print(os.getpid())
      #  print(pid2)
       # os.kill(pid2,signal.SIGINT)
       # os.kill(pid2,signal.SIGTERM)




'''
      *************    EFECTOS SONIDO  APP ***************************
'''

sonidoMouseClick = rutaSonido1
sonidoMouseMove = rutaSonido2

'''
      *************    ESTILOS APP ***************************
'''
estiloBtnPresionado_Verde2 = """
                QWidget { 
                background-color: #494E58; 
                color: white;
                font: 10pt "MS Shell Dlg 2";
                border-radius: 12px;
                border: 0px solid transparent;
                border-left: 5px solid green;
                border-right: 5px solid green;
                }"""

estiloHoverBlanco = """
                QWidget { 
                background-color: #fff;
                
                color: #000;
                font: 400 12pt "Segoe UI";
                border-radius: 4px;
                border: 3px solid #568af2;               
                 }"""

estiloBtnNormal2 = """
                QWidget { 
                    
                color: white;
                background-color: rgb(78, 154, 6); 
                font: 10pt "MS Shell Dlg 2";}"""

estiloBtnSalir = """
                QWidget { 
                background-color: red;
                color: white;
                }
             
             
                QToolTip { 
                background-color: #1b1e23; 
                color: #8a95aa;
                padding-left: 10px;
                padding-right: 10px;
                border-radius: 8px;
                border: 0px solid transparent;
                border-left: 3px solid #568af2;
                font: 800 9pt "Segoe UI";
                min-width: 50px;
                max-width: 50%; 
                }"""

estiloBtnMaximizar = """
                                            
                 QWidget { 
                  background-color: #BEB728;
                  color: white;
                  }
                 
                 
                  QToolTip { 
                background-color: #1b1e23; 
                color: #8a95aa;
                padding-left: 10px;
                padding-right: 10px;
                border-radius: 8px;
                border: 0px solid transparent;
                border-left: 3px solid #568af2;
                font: 800 9pt "Segoe UI";
                min-width: 90px;
                max-width: 90%; }"""

estiloBtnMinimizar = """
                
                QWidget { 
                     background-color: green; 
                     color: white;
                      }
                
                QToolTip { 
                   background-color: #1b1e23; 
                   color: #8a95aa;
                   padding-left: 10px;
                   padding-right: 10px;
                   border-radius: 8px;
                   border: 0px solid transparent;
                   border-left: 3px solid #568af2;
                   font: 800 9pt "Segoe UI";
                   min-width: 90px;
                   max-width: 90%; 
                    }"""

estiloBtnControl = """QWidget { 
                    background-color: rgb(238, 238, 236); 
                    color: white;
                  
                     }"""

estiloBtnCamaraON = """                                        
                    QWidget { 
                    background-color: #3EAC30;
                    color: white;
                    font: 800 12pt "Segoe UI";
                    }
                    """

estiloBtnCamaraOFF = """                                        
                    QWidget { 
                    background-color: red;
                    color: white;
                    font: 800 10pt "Segoe UI";
                    }
                    """

# MAIN WINDOW
# ///////////////////////////////////////////////////////////////
class ventanaPrincipal(QMainWindow):

    def __init__(self):
        
        super(ventanaPrincipal, self).__init__()
        
        self.setWindowTitle("ROS BRAZO ROBOTICO")
        self.procesoROS = None
        ruta = os.path.join(rp.get_path('brazo_drone'), 'src/guide/ventanas', 'main.ui')
        self.ui = loadUi(ruta, self)
        self.setWindowFlags(Qt.FramelessWindowHint)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        self.shadow = QGraphicsDropShadowEffect(self)
        self.shadow.setBlurRadius(180)
        self.shadow.setXOffset(0)
        self.shadow.setYOffset(0)
       
        self.setGraphicsEffect(self.shadow)
        
        self.centrarVentana()

        self.btnExit.installEventFilter(self)
        self.btnMinimizar.installEventFilter(self)
        self.btnEnviar.installEventFilter(self)
        self.btnConexion.installEventFilter(self)
        self.btnSecuencia.installEventFilter(self)
        self.btnCamara.installEventFilter(self)
        
        self.editCOM.setStyleSheet(estiloHoverBlanco)
       
        
        self.estadoGrafica = False
        self.puertoSerial = None
        self.boolBrazo = False
        self.boolRobot = False
        
        
        self.btnCamara.clicked.connect(self.encenderCamara)
        self.btnAbrir.clicked.connect(self.funcionAbrir)
        self.btnCerrar.clicked.connect(self.funcionCerrar)
        self.btnSecuencia.clicked.connect(self.funcionSecuencia)
        self.btnEnviar.clicked.connect(self.funcionEnvio)
        self.btnConexion.clicked.connect(self.funcionRobot)
        
        self.checkBrazo.stateChanged.connect(self.brazoRobot)
        
        self.eslabon1.setSingleStep(1)
        self.eslabon1.sliderReleased.connect(self.funcionEslabon1)

        self.eslabon2.setSingleStep(1)
        self.eslabon2.sliderReleased.connect(self.funcionEslabon2)

        self.eslabon3.setSingleStep(1)
        self.eslabon3.sliderReleased.connect(self.funcionEslabon3)

        self.eslabon4.setSingleStep(1)
        self.eslabon4.sliderReleased.connect(self.funcionEslabon4)

        self.pinza.setSingleStep(1)
        self.pinza.sliderReleased.connect(self.funcionPinza)
        

        
        self.IDProceso = None
       
       
        self.contPuertos=0
        self.ON = False
       
       
        self.findPuertos()
     
     
        
        self.lbl_estado.setText("ROBOT DESCONECTADO")
        self.lbl_estado.setStyleSheet('font: 15pt "Siemens AD Sans";color: red')


        def moveWindow(e):

            if self.isMaximized() == False:
                if e.buttons() == Qt.LeftButton:
                    self.move(self.pos() + e.globalPos() - self.clickPosition)
                    self.clickPosition = e.globalPos()
                    e.accept()

        self.header.mouseMoveEvent = moveWindow
        self.btnExit.clicked.connect(self.funcionSalirApp)
        self.btnMinimizar.clicked.connect(lambda: (self.showMinimized(), self.tonoClick()))


   

        self.funcionInicio()

        time.sleep(2)


        rospy.init_node('PYTHON_GUIDE')
        self.control_publisher = rospy.Publisher('/joint_states', JointState, queue_size=10)
        self.control_visionSMS = rospy.Publisher('SMS_VISION_ARTIFICIAL', String, queue_size=10)
        self.control_vision = rospy.Publisher('IMAGEN_OPENCV', Image, queue_size=10)
        self.control_robot = rospy.Publisher('ROBOT_DRONE', Image, queue_size=10)
        
        self.VARSMS = String()

        robotSIM = QTimer(self)
        robotSIM.timeout.connect(self.funcionSIM)
        robotSIM.start(1)

    def funcionRobot(self):


        global DEVICENAME, PROTOCOL_VERSION, ADDR_TORQUE_ENABLE


        try: 

            DEVICENAME = self.editCOM.toPlainText()
            print("PUERTO: "+DEVICENAME + "   PROTOCOLO: "+ str(PROTOCOL_VERSION))

            

            if (DEVICENAME not in "ttyUSB0"):



                print("SI HAY PUERTO")
            
                self.portHandler = PortHandler(DEVICENAME)
                self.packetHandler = PacketHandler(PROTOCOL_VERSION)

                

                print(self.portHandler)
                print(self.packetHandler)


                try:
                    self.portHandler.openPort()
                    print("Succeeded to open the port")
                except:
                    print("Failed to open the port")
        

                # Set port baudrate
                try:
                    self.portHandler.setBaudRate(BAUDRATE)
                    print("Succeeded to change the baudrate")
                except:
                    print("Failed to change the baudrate")
                    

                        
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, 11, ADDR_TORQUE_ENABLE, 1)
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, 12, ADDR_TORQUE_ENABLE, 1)
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, 13, ADDR_TORQUE_ENABLE, 1)
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, 14, ADDR_TORQUE_ENABLE, 1)
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, 15, ADDR_TORQUE_ENABLE, 1)
                
                self.boolRobot = True
                self.lbl_estado.setText("ROBOT CONECTADO")
                self.lbl_estado.setStyleSheet('font: 15pt "Siemens AD Sans";color: green')



            else:

                QMessageBox.warning(self, 'Error', 'El robot no es compatible')
                print ("NO SE CONECTO")
                self.boolRobot = False
                self.lbl_estado.setText("ROBOT DESCONECTADO")
                self.lbl_estado.setStyleSheet('font: 15pt "Siemens AD Sans";color: red')


        except:

            QMessageBox.warning(self, 'Error', 'Revisar puerto USB')
            print ("ERROR PUERTO")

        
    def secuenciaRobot(self):

        velocidad = 100

        if(self.boolBrazo):
            grados = 2000
            print("SERVO 12")

            for cuenta in range(21):

                    cuenta = cuenta*5
                    grados -= cuenta
                    mm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 12, ADDR_GOAL_POSITION, grados)
                    print(grados)

                    QtTest.QTest.qWait(velocidad)

                   
        
        QtTest.QTest.qWait(1000)

        print("SERVO 13")
        grados = 3000
        for cuenta in range(10):

                cuenta = cuenta*5
                grados -= cuenta
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 13, ADDR_GOAL_POSITION, grados)
                print(grados)
                QtTest.QTest.qWait(100)


        QtTest.QTest.qWait(1000)
        print("SERVO 11")
        grados = 2000
        for cuenta in range(15):

                cuenta = cuenta*5
                grados -= cuenta
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 11, ADDR_GOAL_POSITION, grados)
                print(grados)
                QtTest.QTest.qWait(100)
        
        QtTest.QTest.qWait(1000)
        print("PINZA")
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 15, ADDR_GOAL_POSITION, 3000)

        QtTest.QTest.qWait(1000)
        print("SERVO 13")
        grados = 2775

        for cuenta in range(12):

                cuenta = cuenta*5
                grados += cuenta
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 13, ADDR_GOAL_POSITION, grados)
                print(grados)
                QtTest.QTest.qWait(100)

        QtTest.QTest.qWait(1000)
        print("PINZA")
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 15, ADDR_GOAL_POSITION, 3800)

        QtTest.QTest.qWait(1000)

        print("SERVO 13")
        grados = 3105

        for cuenta in range(12):

                cuenta = cuenta*5
                grados -= cuenta
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 13, ADDR_GOAL_POSITION, grados)
                print(grados)
                QtTest.QTest.qWait(100)

        print("SERVO 11")

        QtTest.QTest.qWait(1000)
        
        grados = 1475
        for cuenta in range(21):

                cuenta = cuenta*5
                grados += cuenta
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 11, ADDR_GOAL_POSITION, grados)
                print(grados)
                QtTest.QTest.qWait(100)


        print("SERVO 13")
        grados = 2775

        QtTest.QTest.qWait(1000)

        for cuenta in range(12):

                cuenta = cuenta*5
                grados += cuenta
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 13, ADDR_GOAL_POSITION, grados)
                print(grados)
                QtTest.QTest.qWait(100)

        QtTest.QTest.qWait(1000)
        print("PINZA")
        
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 15, ADDR_GOAL_POSITION, 3000)
        
        QtTest.QTest.qWait(1000)

        print("SERVO 13")
        grados = 3105

        for cuenta in range(15):

                cuenta = cuenta*5
                grados -= cuenta
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 13, ADDR_GOAL_POSITION, grados)
                print(grados)
                QtTest.QTest.qWait(100)  


        print("PINZA")

        QtTest.QTest.qWait(1000)
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 15, ADDR_GOAL_POSITION, 4000)
        

        QtTest.QTest.qWait(1000)

        print("SERVO 11")

        grados = 2525
        for cuenta in range(15):

                cuenta = cuenta*5
                grados -= cuenta
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 11, ADDR_GOAL_POSITION, grados)
                print(grados)
                QtTest.QTest.qWait(100)


        QtTest.QTest.qWait(1000)

        print("SERVO 13")

        grados = 2580

        for cuenta in range(14):

                cuenta = cuenta*5
                grados += cuenta
                dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 13, ADDR_GOAL_POSITION, grados)
                print(grados)
                QtTest.QTest.qWait(100)  


        

        

    def brazoRobot(self, estado):

        if (estado == QtCore.Qt.Checked):
            self.boolBrazo = True

        else:
            self.boolBrazo = False

        print("brazo: ")
        print(self.boolBrazo)
    

    def funcionSecuencia(self):

        global datoEslabon1, datoEslabon2, datoEslabon3, datoEslabon4, datoPinza
        
        cuenta = 0
        velocidad = 5
        self.funcionEnvio()

        if (self.boolRobot):
            self.secuenciaRobot()

        for cuenta in range(91):
            cuenta = cuenta*-1
            self.eslabon2.setValue(cuenta)
            self.txtLink2.setText(str(cuenta))
            nuevoValor = self.mapeo(cuenta, -90, 90, -1.5, 1.5)
            datoEslabon2 = round(nuevoValor,2)
            QtTest.QTest.qWait(velocidad)
            

        cuenta = 0
        
        for cuenta in range(41):
            self.eslabon3.setValue(cuenta)
            self.txtLink3.setText(str(cuenta))
            nuevoValor = self.mapeo(cuenta, -90, 90, -1.5, 1.5)
            datoEslabon3 = round(nuevoValor,2)
            QtTest.QTest.qWait(velocidad)
      
        cuenta = 0

        for cuenta in range(91):
            cuenta = cuenta*-1
            self.eslabon1.setValue(cuenta)
            self.txtLink1.setText(str(cuenta))
            nuevoValor = self.mapeo(cuenta, -90, 90, -1.5, 1.5)
            datoEslabon1 = round(nuevoValor,2)
            QtTest.QTest.qWait(velocidad)

        QtTest.QTest.qWait(500)
        datoPinza = 0.01
        self.pinza.setValue(90)
        self.txtPinza.setText("90")
        QtTest.QTest.qWait(500)

        cuenta = 0

        for cuenta in range(71):           
            self.eslabon4.setValue(cuenta)
            self.txtLink4.setText(str(cuenta))
            nuevoValor = self.mapeo(cuenta, -90, 90, -1.5, 1.5)
            datoEslabon4 = round(nuevoValor,2)
            QtTest.QTest.qWait(velocidad)

        cuentaI = -90
        cuenta = 0

        for cuenta in range(0,21):            
            cuentaI += 1
            print("Cuenta: "+str(cuenta) + "  angulo: "+str(cuentaI))
            self.eslabon2.setValue(cuentaI)
            self.txtLink2.setText(str(cuenta))
            nuevoValor = self.mapeo(cuentaI, -90, 90, -1.5, 1.5)
            datoEslabon2 = round(nuevoValor,2)
            QtTest.QTest.qWait(velocidad)
        
        QtTest.QTest.qWait(500)
        datoPinza = -0.01
        self.pinza.setValue(0)
        self.txtPinza.setText("0")
        QtTest.QTest.qWait(500)

        cuentaI = -70

        for cuenta in range(0,21):            
            cuentaI -= 1
            print("Cuenta: "+str(cuenta) + "  angulo: "+str(cuentaI))
            self.eslabon2.setValue(cuentaI)
            self.txtLink2.setText(str(cuenta))
            nuevoValor = self.mapeo(cuentaI, -90, 90, -1.5, 1.5)
            datoEslabon2 = round(nuevoValor,2)
            QtTest.QTest.qWait(velocidad)


        cuentaI = -90

        
        for cuenta in range(181):

            cuentaI += 1
            
            self.eslabon1.setValue(cuentaI)
            self.txtLink1.setText(str(cuentaI))
            nuevoValor = self.mapeo(cuentaI, -90, 90, -1.5, 1.5)
            datoEslabon1 = round(nuevoValor,2)
            QtTest.QTest.qWait(velocidad)

        cuentaI = -90

        for cuenta in range(0,21):            
            cuentaI += 1
            print("Cuenta: "+str(cuenta) + "  angulo: "+str(cuentaI))
            self.eslabon2.setValue(cuentaI)
            self.txtLink2.setText(str(cuenta))
            nuevoValor = self.mapeo(cuentaI, -90, 90, -1.5, 1.5)
            datoEslabon2 = round(nuevoValor,2)
            QtTest.QTest.qWait(velocidad)
        
        QtTest.QTest.qWait(500)
        datoPinza = 0.01
        self.pinza.setValue(90)
        self.txtPinza.setText("90")
        QtTest.QTest.qWait(500)


        print("Termino Secuencia Simulada")



    def funcionSIM(self):

        global datoEslabon1, datoEslabon2, datoEslabon3, datoEslabon4, datoPinza

        msg = JointState()

        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = ''
        msg.name = ['propeller_joint1', 'propeller_joint2',  'propeller_joint3', 'propeller_joint4', 'joint1', 'joint2', 'joint3', 'joint4', 'gripper', 'gripper_sub' ]



        msg.position  = [0, 0, 0, 0, datoEslabon1, datoEslabon2, datoEslabon3, datoEslabon4, datoPinza, datoPinza]
        msg.velocity = []
        msg.effort = []

        #point.time_from_start = rospy.Duration(1)

        #msg.points.append(point)

        self.control_publisher.publish(msg)
        

    def funcionAbrir(self):
        global datoPinza 
        datoPinza = 0.01

        if(self.boolRobot):
            mm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 15, ADDR_GOAL_POSITION, 3000)
                   



    def funcionCerrar(self):
        global datoPinza 
        datoPinza = -0.01

        if(self.boolRobot):
            mm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, 15, ADDR_GOAL_POSITION, 4000)
        




    def callback(self, image_msg):
        try:

                cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

                #cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

                (rows,cols,channels) = cv_image.shape
        

                #cv2.imshow("Image window", cv_image)

                self.mostrarVideo(cv_image)
                

        except CvBridgeError as e:
            print(e)


        
        #cv2.waitKey(1)



    def funcionInicio(self):

        if self.procesoROS is None:

            print("Ejecutando simulador brazo...")
            self.procesoROS = QProcess(self)
            programa = 'roslaunch open_manipulator_description open_manipulator_rviz.launch use_gui:=true'

            self.procesoROS.start(programa)
            #self.procesoROS.waitForStarted(1000)

        
        
    def funcionEnvio(self):

        self.tonoClick()


        global datoEslabon1, datoEslabon2, datoEslabon3, datoEslabon4, datoPinza

        datoEslabon1 = 0.0
        datoEslabon2 = 0.0
        datoEslabon3 = 0.0
        datoEslabon4 = 0.0
        datoPinza = 0.0


        self.eslabon1.setValue(int(datoEslabon1))
        self.eslabon2.setValue(int(datoEslabon2))
        self.eslabon3.setValue(int(datoEslabon3))
        self.eslabon4.setValue(int(datoEslabon4))
        self.pinza.setValue(int(datoPinza))
        
        self.txtLink1.setText(str(int(datoEslabon1)))
        self.txtLink2.setText(str(int(datoEslabon2)))
        self.txtLink3.setText(str(int(datoEslabon3)))
        self.txtLink4.setText(str(int(datoEslabon4)))
        self.txtPinza.setText(str(int(datoPinza)))
        
        
        
        
        
    

    def funcionEslabon1(self):

        global datoEslabon1
        
      
        value = self.eslabon1.value()
        nuevoValor = self.mapeo(value, -90, 90, -1.5, 1.5)
        datoEslabon1 = round(nuevoValor,2)
        self.txtLink1.setText(str(value))


    def funcionEslabon2(self):

        global datoEslabon2


        value = self.eslabon2.value()
        nuevoValor = self.mapeo(value, -90, 90, -1.5, 1.5)
        datoEslabon2 = round(nuevoValor,2)

        
        self.txtLink2.setText(str(value))

    
    def funcionEslabon3(self):

        global datoEslabon3

        value = self.eslabon3.value()
        nuevoValor = self.mapeo(value, -90, 90, -1.5, 1.5)
        datoEslabon3 = round(nuevoValor,2)

         
        self.txtLink3.setText(str(value))

    
    def funcionEslabon4(self):

        global datoEslabon4

        value = self.eslabon4.value()
        nuevoValor = self.mapeo(value, -90, 90, -1.5, 1.5)
        datoEslabon4 = round(nuevoValor,2)

        
        self.txtLink4.setText(str(value))

    
    def funcionPinza(self):

        global datoPinza
        

        value = self.pinza.value()
        nuevoValor = self.mapeo(value, 0, 90, -0.01, 0.01)
        datoPinza = round(nuevoValor,3)

        
        self.txtPinza.setText(str(value))



    def mapeo(self,x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        
     

    def findPuertos(self):
        
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
           # print(p.device)
            if "CH340" in p.description or "Arduino" in p.description or "ttyACM0" in p.description or "ttyUSB0" in p.description or  "/dev/ttyUSB0" in p.device:
                 print (p.device)
                 self.editCOM.setText(p.device)
                 self.contPuertos=self.contPuertos + 1


    def centrarVentana(self):

        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())
        #return container
        

    def funcionSalirApp(self):
        self.tonoClick()

        if(self.ON):
            self.detenerVideo()

        if self.procesoROS is not None:

            print("Proceso Terminado")
        #self.procesoROS.terminate()
        #self.procesoROS.waitForFinished()
        #os.kill(self.IDProceso,signal.SIGINT)


            process_name1 = "rviz"
            process_name2 = "roslaunch"
            #process_name3 = "rosmaster"

        

            for proc in psutil.process_iter():
                if process_name1 in proc.name():

                #print(proc.name())
                    pid1 = proc.pid
                
                    print(pid1)
                    os.kill(pid1,signal.SIGINT)
                    os.kill(pid1,signal.SIGTERM)

                if process_name2 in proc.name():
                
                    pid2 = proc.pid
                    #print(os.getpid())
                    print(pid2)
                    os.kill(pid2,signal.SIGINT)
                    os.kill(pid2,signal.SIGTERM)

               # if process_name3 in proc.name():
                
                  #  pid3 = proc.pid
                    #print(os.getpid())
                #    print(pid3)
                  #  os.kill(pid3,signal.SIGINT)
                   # os.kill(pid3,signal.SIGTERM)


            #self.procesoROS.kill()
            self.procesoROS = None
            #time.sleep(1)
            self.close()


            #self.procesoROS.finished.connect(self.terminarProceso)


        


    def tonoClick(self):
        mixer.init()
        mixer.music.load(sonidoMouseClick)
        mixer.music.play()
    
    def tonoMenu(self):
        mixer.init()
        mixer.music.load(sonidoMouseMove)
        mixer.music.play()
    
    def mousePressEvent(self, event):
        self.clickPosition = event.globalPos()
           
    def eventFilter(self, object, event):

        if object is self.btnCamara and event.type() == QtCore.QEvent.Enter:
            self.tonoMenu()
            if (self.ON == False):
                self.btnCamara.setText("ENCENDER CÁMARA")
                self.btnCamara.setStyleSheet(estiloBtnCamaraON)
            else:
                self.btnCamara.setStyleSheet(estiloBtnCamaraOFF)
                self.btnCamara.setText("APAGAR CÁMARA")
            return True

        elif object is self.btnCamara and event.type() == QtCore.QEvent.Leave:
            if self.ON == False:
                self.btnCamara.setStyleSheet(estiloBtnNormal2)
            else:
                self.btnCamara.setStyleSheet(estiloBtnPresionado_Verde2)
                
       
             
        
        if object is self.btnEnviar and event.type() == QtCore.QEvent.Enter:
             self.tonoMenu()
             self.btnEnviar.setStyleSheet(estiloBtnCamaraON)
             return True

        elif object is self.btnEnviar and event.type() == QtCore.QEvent.Leave:
             self.btnEnviar.setStyleSheet(estiloBtnNormal2)


        
        if object is self.btnConexion and event.type() == QtCore.QEvent.Enter:
             self.tonoMenu()
             self.btnConexion.setStyleSheet(estiloBtnCamaraON)
             return True

        elif object is self.btnConexion and event.type() == QtCore.QEvent.Leave:
             self.btnConexion.setStyleSheet(estiloBtnNormal2)

            
        
        if object is self.btnSecuencia and event.type() == QtCore.QEvent.Enter:
             self.tonoMenu()
             self.btnSecuencia.setStyleSheet(estiloBtnCamaraON)
             return True

        elif object is self.btnSecuencia and event.type() == QtCore.QEvent.Leave:
             self.btnSecuencia.setStyleSheet(estiloBtnNormal2)


               
       
        if object is self.btnExit and event.type() == QtCore.QEvent.Enter:
            self.tonoMenu()
            self.btnExit.setStyleSheet(estiloBtnSalir)
            self.btnExit.setToolTip("Salir")

            imgW = os.path.join(rp.get_path('brazo_drone'), 'src/guide/iconos', 'icon_closeW.svg')
            self.btnExit.setIcon(QIcon(imgW))
            return True

        elif object is self.btnExit and event.type() == QtCore.QEvent.Leave:

            imgN = os.path.join(rp.get_path('brazo_drone'), 'src/guide/iconos', 'icon_close.svg')
            self.btnExit.setIcon(QIcon(imgN))

            self.btnExit.setStyleSheet(estiloBtnControl)
            
            
       
        if object is self.btnMinimizar and event.type() == QtCore.QEvent.Enter:
            self.tonoMenu()
            self.btnMinimizar.setStyleSheet(estiloBtnMinimizar)
            self.btnMinimizar.setToolTip("Minimizar")

            imgW = os.path.join(rp.get_path('brazo_drone'), 'src/guide/iconos', 'icon_minimizeW.svg')
            self.btnMinimizar.setIcon(QIcon(imgW))

            
            return True

        elif object is self.btnMinimizar and event.type() == QtCore.QEvent.Leave:
            self.btnMinimizar.setStyleSheet(estiloBtnControl)
            imgN = os.path.join(rp.get_path('brazo_drone'), 'src/guide/iconos', 'icon_minimize.svg')
            self.btnMinimizar.setIcon(QIcon(imgN))
        return False
    
    
    def encenderCamara(self):
         
             
        self.tonoClick()
            
        if (self.ON == False):
            #self.puertoSerial = serial.Serial(self.editCOM.toPlainText(), 9600, timeout=0.01)
            #time.sleep(2)  # Espera 2 segundos para conectar puerto serial
            #self.puertoSerial.flushInput()
            self.btnCamara.setStyleSheet(estiloBtnPresionado_Verde2)
            self.btnCamara.setText("APAGAR CÁMARA")
            self.startCamara()
            
            
        # self.btnConexion.setIcon(QIcon('imagenes/guide/button_apagar-camara.png'))
        else:
            self.btnCamara.setText("ENCENDER CÁMARA")
            self.btnCamara.setStyleSheet(estiloBtnNormal2)
            self.detenerVideo()
            self.ON = False
         

    def detenerVideo(self):
        global i, y1, y2, x, valSetPoint
        
        self.lblVideo.setVisible(False)
        
        
        #self.puertoSerial.close()
        #self.puertoSerial = None
        #self.ON = False
        
        self.timer.stop()
        
        #self.lbl_estado.setText("DISPOSITIVO DESCONECTADO")
       # self.lbl_estado.setStyleSheet('font: 15pt "Siemens AD Sans";color: red')
        
        
        
        

    def startCamara(self):
        try:
            # Node is publishing to the video_frames topic using 
            # the message type Image

            #rospy.init_node('video_SEBAS_py', anonymous=True)

            

            # Tells rospy the name of the node.
            # Anonymous = True makes sure the node has a unique name. Random
            # numbers are added to the end of the name.
            

            # Go through the loop 10 times per second
            self.rate = rospy.Rate(100) # 10hz

            # Create a VideoCapture object
            # The argument '0' gets the default webcam.
            #cap = cv2.VideoCapture(0)
            self.cap = cv2.VideoCapture(0)

            # Used to convert between ROS and OpenCV images
            self.br = CvBridge()

            self._cv_bridge = CvBridge()


            ret, ima1 = self.cap.read()

            self.height, self.width, self.channels = ima1.shape


            self.lblVideo.setVisible(True)
            self.ON = True      
           

            print("Encendiendo cámara")


            #self._cv_bridge = CvBridge()
            self._sub = rospy.Subscriber('IMAGEN_OPENCV', Image, self.callback, queue_size=10)
            
            
            self.timer = QTimer(self)
            self.timer.timeout.connect(self.loopCamara)
            self.timer.start(0)
            
        except:
             QMessageBox.warning(self, 'Error', 'Conecte la cámara')
             self.ON = False
            
             self.puertoSerial.close()
             self.puertoSerial = None
        
    def loopCamara(self):
        
            #try:    
            self.publish_message()

            #except:
             #   self.detenerVideo()


    def publish_message(self):

        
        # While ROS is still running.
        

            # Capture frame-by-frame
            # This method returns True/False as well
            # as the video frame.
        ret, frame = self.cap.read()

        if ret:


            hands, frame = detector.findHands(frame)  # with draw
    # hands = detector.findHands(img, draw=False)  # without draw

            if hands:
                # Hand 1
                hand1 = hands[0]
                lmList1 = hand1["lmList"]  # List of 21 Landmark points
                bbox1 = hand1["bbox"]  # Bounding box info x,y,w,h
                centerPoint1 = hand1['center']  # center of the hand cx,cy
                handType1 = hand1["type"]  # Handtype Left or Right

                fingers1 = detector.fingersUp(hand1)
                #print(fingers1)

                if(fingers1 == [1, 1, 1, 1, 1]):
                    #print("OBJETO DETECTADO!!!")
                    self.VARSMS.data ="OBJETO DETECTADO!!!"
                    self.control_visionSMS.publish(self.VARSMS)
                    cv2.putText(frame, "OBJETO DETECTADO!!!", (self.width-370, 60), cv2.FONT_HERSHEY_COMPLEX, 1, (0,0,255), 5)
                    cv2.putText(frame, "OBJETO DETECTADO!!!", (self.width-370, 60), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255), 2)
                    


                if len(hands) == 2:
                    # Hand 2
                    hand2 = hands[1]
                    lmList2 = hand2["lmList"]  # List of 21 Landmark points
                    bbox2 = hand2["bbox"]  # Bounding box info x,y,w,h
                    centerPoint2 = hand2['center']  # center of the hand cx,cy
                    handType2 = hand2["type"]  # Hand Type "Left" or "Right"

                    fingers2 = detector.fingersUp(hand2)
                    print(fingers2)

                    

                    # Find Distance between two Landmarks. Could be same hand or different hands
                    #length, info, frame= detector.findDistance(lmList1[8], lmList2[8])  # with draw

            else:
                self.VARSMS.data ="SIN DETECCION"
                self.control_visionSMS.publish(self.VARSMS)
                    
            rospy.loginfo('publishing video frame')
            #self.mostrarVideo(frame)

                # Publish the image.
                # The 'cv2_to_imgmsg' method converts an OpenCV
                # image to a ROS image message
                
            self.control_vision.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))

            #pub.publish(br.cv2_to_imgmsg(frame, encoding="passthrough"))


            # Sleep just enough to maintain the desired rate
            #self.rate.sleep()

            #if cv2.waitKey(1) & 0xFF == ord('q'): 
                #break


    
    def mostrarVideo(self, img):

        qformat = QImage.Format_Indexed8
        if len(img.shape) == 2:
            qformat = QImage.Format_Grayscale8
        if len(img.shape) == 3:
            if img.shape[2] == 4:
                qformat = QImage.Format_RGBA8888
            else:
                qformat = QImage.Format_RGB888
        outimg = QImage(img, img.shape[1], img.shape[0], img.strides[0], qformat)
        outimg = outimg.rgbSwapped()
        self.lblVideo.setPixmap(QPixmap.fromImage(outimg))
        self.lblVideo.setScaledContents(True)


       
        


if __name__ == "__main__":
    # APPLICATION
    # ///////////////////////////////////////////////////////////////
    app = QApplication(sys.argv)
    app.setWindowIcon(QIcon("icon.ico"))
    window = ventanaPrincipal()
    window.show()
    # EXEC APP
    # ///////////////////////////////////////////////////////////////
    sys.exit(app.exec_())