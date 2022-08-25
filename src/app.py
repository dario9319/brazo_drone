# -*- coding: utf-8 -*-
"""
Created on Mon Jan 24 09:52:32 2022

@author: sebas
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

from pygame import mixer  # Load the popular external library

from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

import psutil
import signal

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

from cvzone.HandTrackingModule import HandDetector

detector = HandDetector(detectionCon=0.8, maxHands=2)


datoEslabon1 = 0.0
datoEslabon2 = 0.0
datoEslabon3 = 0.0
datoEslabon4 = 0.0


rp = RosPack()
rutaSonido2 = os.path.join(rp.get_path('brazo_drone'), 'src/guide/sonidos', 'cursor-move.mp3')
rutaSonido1 = os.path.join(rp.get_path('brazo_drone'), 'src/guide/sonidos', 'click.mp3')


process_name1 = "gzserver"
process_name2 = "gzclient"
process_name3 = "roslaunch"
process_name4 = "rosmaster"


pid = None


for proc in psutil.process_iter():
    #if process_name in proc.name():

    #print(proc.name())
    if process_name1 in proc.name():
        pid1 = proc.pid
        #print(os.getpid())

        print(pid1)
        os.kill(pid1,signal.SIGINT)
        os.kill(pid1,signal.SIGTERM)

    if process_name2 in proc.name():
     
        pid2 = proc.pid
        #print(os.getpid())
        print(pid2)
        os.kill(pid2,signal.SIGINT)
        os.kill(pid2,signal.SIGTERM)

    
    if process_name3 in proc.name():
     
        pid2 = proc.pid
        #print(os.getpid())
        print(pid2)
        os.kill(pid2,signal.SIGINT)
        os.kill(pid2,signal.SIGTERM)

    
    if process_name4 in proc.name():
     
        pid2 = proc.pid
        #print(os.getpid())
        print(pid2)
        os.kill(pid2,signal.SIGINT)
        os.kill(pid2,signal.SIGTERM)
        



'''
      *************    EFECTOS SONIDO  APP ***************************
'''
# sonidoMouseMove  = 'guide/sonidos/hover.mp3'
sonidoMouseClick = rutaSonido1
sonidoMouseMove = rutaSonido2
# sonidoMouseClick = 'guide/sonidos/cursor-select.mp3'

'''
      *************    ESTILOS APP ***************************
'''
estiloBtnPresionado_Blue = """QWidget { 
                background-color:#3c4454; 
                color: white;
                font: 8pt "MS Shell Dlg 2";
                border-radius: 12px;
                border: 0px solid transparent;
                border-left: 3px solid #568af2;}"""

estiloBtnPresionado_Verde = """
                QWidget { 
                background-color:#3c4454; 
                color: white;
                font: 8pt "MS Shell Dlg 2";
                border-radius: 12px;
                border: 0px solid transparent;
                border-left: 3px solid green;}"""

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

estiloBtnPresionado_Capturar = """
                QWidget { 
                background-color: #494E58; 
                color: white;
                font: 10pt "MS Shell Dlg 2";
                border-radius: 12px;
                border: 0px solid transparent;
                border-left: 5px solid #BEB728;
                border-right: 5px solid #BEB728;
                }"""

estiloHoverGris = """
                QWidget { 
                background-color: #3c4454;
                
                color: white;
                font: 800 9pt "Segoe UI";
                }"""

estiloHoverAzul = """
                QWidget { 
                background-color: #568af2;
                
                color: white;
                font: 800 9pt "Segoe UI";
                 }"""

estiloHoverBlanco = """
                QWidget { 
                background-color: #fff;
                
                color: #000;
                font: 400 12pt "Segoe UI";
                border-radius: 4px;
                border: 3px solid #568af2;               
                 }"""

estiloHoverMuestras = """
                QWidget { 
                background-color: #3c4454;               
                color: #fff;
                font: 500 16pt "Segoe UI";
                border-radius: 4px;
                border: 3px solid #568af2;               
                 }"""

estiloHoverVerde = """
                QWidget { 
                background-color: #3EAC30;
                
                color: white;
                font: 800 9pt "Segoe UI";
                 }"""

estiloSinBordeIzquierdo = """
                 QWidget { 
                 border-left: 0px; }"""

estiloBtnNormal = """
                QWidget { 
                background-color: #1b1e23; 
                font: 8pt "MS Shell Dlg 2";}"""

estiloBtnNormal2 = """
                QWidget { 
                    
                color: white;
                background-color: #1b1e23; 
                font: 10pt "MS Shell Dlg 2";}"""

estiloTooltip = """
                QToolTip { 
                background-color: #1b1e23; 
                color: #8a95aa;
                padding-left: 10px;
                padding-right: 10px;
                border-radius: 8px;
                border: 0px solid transparent;
                border-left: 3px solid #568af2;
                font: 800 9pt "Segoe UI";
                min-width: 120px;
                max-width: 120%; 
                 }"""

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
                    background-color: #343b48; 
                    color: white;
                  
                     }"""

estiloTooltip100 = """QToolTip { 
                    background-color: #1b1e23; 
                    color: #8a95aa;
                    padding-left: 10px;
                    padding-right: 10px;
                    border-radius: 8px;
                    border: 0px solid transparent;
                    border-left: 3px solid #568af2;
                    font: 800 9pt "Segoe UI";
                    min-width: 100px;
                    max-width: 90%; 
                     }"""

estiloTooltip70 = """QToolTip { 
                    background-color: #1b1e23; 
                    color: #8a95aa;
                    padding-left: 10px;
                    padding-right: 10px;
                    border-radius: 8px;
                    border: 0px solid transparent;
                    border-left: 3px solid #568af2;
                    font: 800 9pt "Segoe UI";
                    min-width: 70px;
                    max-width: 70%; 
                     }"""

estiloBtnCamaraON = """                                        
                    QWidget { 
                    background-color: #3EAC30;
                    color: white;
                    font: 800 10pt "Segoe UI";
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
        
        
        self.editCOM.setStyleSheet(estiloHoverBlanco)
       
        
        self.estadoGrafica = False
        self.puertoSerial = None
        self.btnConexion.installEventFilter(self)
        
        
        #self.btnConexion.clicked.connect(self.encenderCamara)
        self.btnCamara.clicked.connect(self.encenderCamara)


        self.btnInicio.clicked.connect(self.funcionInicio)
        
      
        self.btnEnviar.clicked.connect(self.funcionEnvio)
        
        
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


    

    def callback(self, image_msg):
        try:

                cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

                #cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

                (rows,cols,channels) = cv_image.shape
        

                #cv2.imshow("Image window", cv_image)

                #self.mostrarVideo(cv_image)
                

        except CvBridgeError as e:
            print(e)


        
        #cv2.waitKey(1)



    def funcionInicio(self):

        if self.procesoROS is None:

            print("Ejecutando simulador brazo...")
            self.procesoROS = QProcess(self)

            programa = 'roslaunch open_manipulator_gazebo open_manipulator_gazebo.launch'

            #self.procesoROS.finished.connect(self.terminarProceso)
            
           
            
            #programa = 'roslaunch open_manipulator_controllers joint_trajectory_controller.launch'
            
            self.procesoROS.start(programa)
            #self.procesoROS.waitForStarted(1000)


            procesoROS2 = QProcess(self)

            programa = 'roslaunch open_manipulator_controller open_manipulator_controller.launch use_platform:=false'
            procesoROS2.start(programa)
            procesoROS2.waitForStarted(1000)

            #self.IDProceso = self.procesoROS.pid()

            #print("Proceso ID: "+str(int(self.IDProceso)))




    def terminarProceso(self):


        print("Proceso Terminado")
        self.procesoROS.terminate()
        #self.procesoROS.waitForFinished()
        #os.kill(self.IDProceso,signal.SIGINT)


        process_name1 = "gzserver"
        process_name2 = "gzclient"

       

        for proc in psutil.process_iter():
            #if process_name in proc.name():

            #print(proc.name())
            if process_name1 in proc.name():
                pid1 = proc.pid
                #print(os.getpid())

                print(pid1)
                os.kill(pid1,signal.SIGINT)
                os.kill(pid1,signal.SIGTERM)

            if process_name2 in proc.name():
            
                pid2 = proc.pid
                #print(os.getpid())
                print(pid2)
                os.kill(pid2,signal.SIGINT)
                os.kill(pid2,signal.SIGTERM)


        self.procesoROS.kill()
        self.procesoROS = None


        
        
    def funcionEnvio(self):


        global datoEslabon1, datoEslabon2, datoEslabon3, datoEslabon4, goal_joint_space_path_service_client, goal_joint_space_path_request_object
        
        rospy.init_node('service_set_joint_position_client')
        rospy.wait_for_service('/goal_joint_space_path')
        goal_joint_space_path_service_client = rospy.ServiceProxy('/goal_joint_space_path', SetJointPosition)
        goal_joint_space_path_request_object = SetJointPositionRequest()
        goal_joint_space_path_request_object.planning_group = 'arm'
        goal_joint_space_path_request_object.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4']
        goal_joint_space_path_request_object.joint_position.max_accelerations_scaling_factor = 1.0
        goal_joint_space_path_request_object.joint_position.max_velocity_scaling_factor = 1.0
        goal_joint_space_path_request_object.path_time = 1.0

        

        rospy.loginfo("Doing Service Call...")

        goal_joint_space_path_request_object.joint_position.position = [datoEslabon1, datoEslabon2, datoEslabon3, datoEslabon4]
        result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
        print(result)

        
        
    
    
    

    def funcionEslabon1(self):

        global datoEslabon1,goal_joint_space_path_service_client, goal_joint_space_path_request_object
        
        

        value = self.eslabon1.value()
        nuevoValor = self.mapeo(value, -180, 180, -1.5, 1.5)
        datoEslabon1 = round(nuevoValor,2)
        self.txtLink1.setText(str(value))


        rospy.loginfo("Doing Service Call...")

        goal_joint_space_path_request_object.joint_position.position = [datoEslabon1, datoEslabon2, datoEslabon3, datoEslabon4]
        result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
        print(result)


        
        

    
    def funcionEslabon2(self):

        global datoEslabon2,goal_joint_space_path_service_client, goal_joint_space_path_request_object

       

        value = self.eslabon2.value()
        nuevoValor = self.mapeo(value, -180, 180, -1.5, 1.5)
        datoEslabon2 = round(nuevoValor,2)

        goal_joint_space_path_request_object.joint_position.position = [datoEslabon1, datoEslabon2, datoEslabon3, datoEslabon4]
        result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
        print(result)

        
        
        self.txtLink2.setText(str(value))

    
    def funcionEslabon3(self):

        global datoEslabon3,goal_joint_space_path_service_client, goal_joint_space_path_request_object

        value = self.eslabon3.value()
        nuevoValor = self.mapeo(value, -180, 180, -1.5, 1.5)
        datoEslabon3 = round(nuevoValor,2)

        goal_joint_space_path_request_object.joint_position.position = [datoEslabon1, datoEslabon2, datoEslabon3, datoEslabon4]
        result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
        print(result)
        
        self.txtLink3.setText(str(value))

    
    def funcionEslabon4(self):

        global datoEslabon4,goal_joint_space_path_service_client, goal_joint_space_path_request_object

        value = self.eslabon4.value()
        nuevoValor = self.mapeo(value, -180, 180, -1.5, 1.5)
        datoEslabon4 = round(nuevoValor,2)

        goal_joint_space_path_request_object.joint_position.position = [datoEslabon1, datoEslabon2, datoEslabon3, datoEslabon4]
        result = goal_joint_space_path_service_client(goal_joint_space_path_request_object)
        print(result)
        
        self.txtLink4.setText(str(value))

    
    def funcionPinza(self):


        

        value = self.pinza.value()
        nuevoValor = self.mapeo(value, 0, 90, -0.01, 0.01)
        datoPinza = round(nuevoValor,2)

        rospy.wait_for_service('/goal_tool_control')
        goal_joint_space_path_service_client2 = rospy.ServiceProxy('/goal_tool_control', SetJointPosition)
        goal_joint_space_path_request_object2 = SetJointPositionRequest()

        goal_joint_space_path_request_object2.planning_group = 'gripper'
        goal_joint_space_path_request_object2.joint_position.joint_name = ['gripper']
        goal_joint_space_path_request_object2.joint_position.position = [datoPinza]
        goal_joint_space_path_request_object2.joint_position.max_accelerations_scaling_factor = 1.0
        goal_joint_space_path_request_object2.joint_position.max_velocity_scaling_factor = 1.0
        goal_joint_space_path_request_object2.path_time = 1.0

        rospy.loginfo("Moving Gripper...")

        result = goal_joint_space_path_service_client2(goal_joint_space_path_request_object2)

        print(result)

        
        self.txtPinza.setText(str(value))



    def mapeo(self,x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        
     

    def findPuertos(self):
        
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            print(p)
            if "CH340" in p.description or "Arduino" in p.description or "ttyACM0" in p.description:
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
      
            self.procesoROS.finished.connect(self.terminarProceso)
        self.close()


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
                self.btnCamara.setText("CONECTAR")
                self.btnCamara.setStyleSheet(estiloBtnCamaraON)
            else:
                self.btnCamara.setStyleSheet(estiloBtnCamaraOFF)
                self.btnCamara.setText("DESCONECTAR")
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
               
       
        if object is self.btnExit and event.type() == QtCore.QEvent.Enter:
            self.tonoMenu()
            self.btnExit.setStyleSheet(estiloBtnSalir)
            self.btnExit.setToolTip("Salir")
            return True

        elif object is self.btnExit and event.type() == QtCore.QEvent.Leave:
            self.btnExit.setStyleSheet(estiloBtnControl)
            
            
       
        if object is self.btnMinimizar and event.type() == QtCore.QEvent.Enter:
            self.tonoMenu()
            self.btnMinimizar.setStyleSheet(estiloBtnMinimizar)
            self.btnMinimizar.setToolTip("Minimizar")
            return True

        elif object is self.btnMinimizar and event.type() == QtCore.QEvent.Leave:
            self.btnMinimizar.setStyleSheet(estiloBtnControl)
        return False
    
    
    def encenderCamara(self):
         try:
             
             
            if (self.ON == False):
                #self.puertoSerial = serial.Serial(self.editCOM.toPlainText(), 9600, timeout=0.01)
                #time.sleep(2)  # Espera 2 segundos para conectar puerto serial
                #self.puertoSerial.flushInput()
                self.btnCamara.setStyleSheet(estiloBtnPresionado_Verde2)
                self.tonoClick()
                self.startCamara()
                print("si presione btb")
                
            # self.btnConexion.setIcon(QIcon('imagenes/guide/button_apagar-camara.png'))
            else:
                self.btnCamara.setText("CONECTAR")
                self.btnCamara.setStyleSheet(estiloBtnNormal2)
                self.detenerVideo()
                self.ON = False
         except:
              QMessageBox.warning(self, 'Error', 'Conecte Arduino ')

    def detenerVideo(self):
        global i, y1, y2, x, valSetPoint
        
        self.lblVideo.setVisible(False)
        
        
        #self.puertoSerial.close()
        #self.puertoSerial = None
        #self.ON = False
        
        self.timer.stop()
        
        self.lbl_estado.setText("DISPOSITIVO DESCONECTADO")
        self.lbl_estado.setStyleSheet('font: 15pt "Siemens AD Sans";color: red')
        
        
        
        

    def startCamara(self):
        try:
            # Node is publishing to the video_frames topic using 
            # the message type Image

            rospy.init_node('video_SEBAS_py', anonymous=True)

            #self.pub = rospy.Publisher('video_frames', Image, queue_size=10)

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


            self.lblVideo.setVisible(True)
            self.ON = True      
            self.lbl_estado.setText("DISPOSITIVO CONECTADO")
            self.lbl_estado.setStyleSheet('font: 15pt "Siemens AD Sans";color: green')


            print("Encendiendo cámara")


            #self._cv_bridge = CvBridge()
            #self._sub = rospy.Subscriber('video_frames', Image, self.callback, queue_size=10)
            
            
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

                if len(hands) == 2:
                    # Hand 2
                    hand2 = hands[1]
                    lmList2 = hand2["lmList"]  # List of 21 Landmark points
                    bbox2 = hand2["bbox"]  # Bounding box info x,y,w,h
                    centerPoint2 = hand2['center']  # center of the hand cx,cy
                    handType2 = hand2["type"]  # Hand Type "Left" or "Right"

                    fingers2 = detector.fingersUp(hand2)

                    # Find Distance between two Landmarks. Could be same hand or different hands
                    length, info, frame= detector.findDistance(lmList1[8], lmList2[8])  # with draw


                    
                    #rospy.loginfo('publishing video frame')
            self.mostrarVideo(frame)

                # Publish the image.
                # The 'cv2_to_imgmsg' method converts an OpenCV
                # image to a ROS image message
                #self.pub.publish(self.br.cv2_to_imgmsg(frame, encoding="bgr8"))

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