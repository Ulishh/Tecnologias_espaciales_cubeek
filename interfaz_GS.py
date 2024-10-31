import serial # Libreria para leer el serial
import time # Libreria para timers
import re  # Libreria para usar expresiones regulares
import numpy as np # Libreria para procesar datos
# Librerias para la interfaz
import sys 
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel
from PyQt5 import QtCore, QtGui, QtWidgets
# Librerias para las graficas
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        # Configurar comunicación serial con Arduino
        self.serial_port = serial.Serial('COM10', 9600)  # Cambiar el puerto COM si es necesario
        time.sleep(2)  # Esperar a que el puerto serial se estabilice

        # Definir el tamaño de la ventana y el título
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(600, 600)
        MainWindow.setStyleSheet("background-color: rgb(255, 255, 255);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        # Elementos de la interfaz gráfica
        # Titulo de la interfaz
        self.textEdit = QtWidgets.QTextEdit(self.centralwidget)
        self.textEdit.setGeometry(QtCore.QRect(180, 20, 271, 61))
        self.textEdit.setObjectName("textEdit")
        self.textEdit.setReadOnly(True)
        _translate = QtCore.QCoreApplication.translate
        self.textEdit.setHtml(_translate("MainWindow", 
            "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
            "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
            "p, li { white-space: pre-wrap; }\n"
            "</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8pt; font-weight:400; font-style:normal;\">\n"
            "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt; font-weight:600;\">CUBEEK </span></p>\n"
            "<p align=\"center\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:10pt; font-weight:600;\">GROUND STATION</span></p></body></html>"
        ))
        # Boton para iniciar el modo de programacion
        self.boton_programador = QtWidgets.QPushButton(self.centralwidget)
        self.boton_programador.setGeometry(QtCore.QRect(10, 10, 151, 31))
        self.boton_programador.setObjectName("boton_programador")
        self.boton_programador.clicked.connect(self.modo_programador)
        # Espacio para mostrar si esta activo el modo de programacion
        self.TextEstatus = QtWidgets.QLineEdit(self.centralwidget)
        self.TextEstatus.setGeometry(QtCore.QRect(10, 50, 61, 31))
        self.TextEstatus.setReadOnly(True)
        self.TextEstatus.setObjectName("TextEstatus")
        self.ModoEstatus = QtWidgets.QLineEdit(self.centralwidget)
        self.ModoEstatus.setGeometry(QtCore.QRect(90, 50, 71, 31))
        self.ModoEstatus.setReadOnly(True)
        self.ModoEstatus.setObjectName("ModoEstatus")
        # Subtitulos para las opciones de sensores y tiempo
        self.TextSensores = QtWidgets.QLineEdit(self.centralwidget)
        self.TextSensores.setGeometry(QtCore.QRect(20, 110, 151, 31))
        self.TextSensores.setReadOnly(True)
        self.TextSensores.setObjectName("TextSensores")
        self.TextTiempo = QtWidgets.QLineEdit(self.centralwidget)
        self.TextTiempo.setGeometry(QtCore.QRect(370, 110, 181, 31))
        self.TextTiempo.setReadOnly(True)
        self.TextTiempo.setObjectName("TextTiempo")
        # Botones para el envio de los sensores y tiempo seleccionados
        self.EnvioSensor = QtWidgets.QPushButton(self.centralwidget)
        self.EnvioSensor.setGeometry(QtCore.QRect(60, 200, 61, 31))
        self.EnvioSensor.setObjectName("EnvioSensor")
        self.EnvioSensor.clicked.connect(self.enviar_sensor)
        self.EnvioTiempo = QtWidgets.QPushButton(self.centralwidget)
        self.EnvioTiempo.setGeometry(QtCore.QRect(430, 200, 61, 31))
        self.EnvioTiempo.setObjectName("EnvioTiempo")
        self.EnvioTiempo.clicked.connect(self.enviar_tiempo)
        # Lista desplegable para las opciones de sensores
        self.comboBox = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox.setGeometry(QtCore.QRect(20, 150, 151, 31))
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("a) GPS")
        self.comboBox.addItem("b) IMU")
        self.comboBox.addItem("c) Presión")
        self.comboBox.addItem("d) Luz")
        self.comboBox.addItem("e) Temperatura")
        self.comboBox.addItem("f) Voltaje")
        self.comboBox.addItem("g) Corriente")
        self.comboBox.addItem("s) Cancelar")
        # Lista desplegable para las opciones de tiempo de sensado
        self.comboBox_2 = QtWidgets.QComboBox(self.centralwidget)
        self.comboBox_2.setGeometry(QtCore.QRect(370, 150, 181, 31))
        self.comboBox_2.setObjectName("comboBox_2")
        self.comboBox_2.addItem("a) 20 segundos")
        self.comboBox_2.addItem("b) 40 segundos")
        self.comboBox_2.addItem("c) 1 minuto")
        self.comboBox_2.addItem("s) Cancelar")
        # Espacio para mostrar los datos recibidos por el serial
        self.Datos_serial = QtWidgets.QPlainTextEdit(self.centralwidget)
        self.Datos_serial.setGeometry(QtCore.QRect(10, 260, 271, 301))
        self.Datos_serial.setReadOnly(True)
        self.Datos_serial.setObjectName("Datos_serial")

        # Espacio para la gráfica
        self.graph_widget = QWidget(self.centralwidget)
        self.graph_widget.setGeometry(QtCore.QRect(320, 260, 261, 301))
        self.graph_layout = QVBoxLayout(self.graph_widget)
        # Se crea la figura para la gráfica
        self.figure = plt.figure()
        self.canvas = FigureCanvas(self.figure)
        self.graph_layout.addWidget(self.canvas)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 600, 18))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        # Se inicia un temporizador para leer datos del serial
        self.timer = QTimer()
        self.timer.timeout.connect(self.leer_serial)
        self.timer.start(100)  # Revisar cada 100 ms

        # Listas para almacenar los datos de tiempo (eje X) y los valores de los sensores (eje Y)
        self.tiempo = []
        self.valores = []


    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "CUBEEK"))
        self.boton_programador.setText(_translate("MainWindow", "Modo programador"))
        self.TextEstatus.setText(_translate("MainWindow", "Estatus:"))
        self.TextSensores.setText(_translate("MainWindow", "Sensores"))
        self.TextTiempo.setText(_translate("MainWindow", "Tiempo de sensado"))
        self.EnvioSensor.setText(_translate("MainWindow", "Enviar"))
        self.EnvioTiempo.setText(_translate("MainWindow", "Enviar"))

    # Funcion para activar el modo programador
    def modo_programador(self):
        self.serial_port.write(b'q')  # Enviar 'q' para activar el modo programador
        self.ModoEstatus.setStyleSheet("color: green;")
        self.ModoEstatus.setText("ON")
        self.Datos_serial.clear() # Se borra la grafica 
        # Se vuelven a iniciar las variables vacias para la graficacion
        self.tiempo = []
        self.valores = []
        # Se limpia la figura y redibujar la gráfica
        self.figure.clear()
        self.canvas.draw()
    
    # Función para enviar el sensor seleccionado por el ComboBox
    def enviar_sensor(self):
        self.sensor_opcion = self.comboBox.currentText()[0]  # Extraer la letra seleccionada
        self.serial_port.write(self.sensor_opcion.encode())  # Enviar la opción al Arduino por el serial
        # Condicionales para activar la variable de grafica en tal caso de seleccionar ciertos 
        # sensores que si queremos graficar ya que no graficamos gps, imu y presion.
        if self.sensor_opcion == 'd':
            self.grafica = True
        elif self.sensor_opcion == 'e':
            self.grafica = True
        elif self.sensor_opcion == 'f':
            self.grafica = True
        elif self.sensor_opcion == 'g':
            self.grafica = True
        else:
            self.grafica = False
                
    # Función para enviar el tiempo seleccionado por el ComboBox_2
    def enviar_tiempo(self):
        tiempo_opcion = self.comboBox_2.currentText()[0]  # Extraer la letra seleccionada
        self.serial_port.write(tiempo_opcion.encode())  # Enviar la opción al Arduino por el serial
        self.ModoEstatus.setStyleSheet("color: red;")
        self.ModoEstatus.setText("OFF") 

    # Función para leer datos del puerto serial, es decir, los datos que recibimos del satelite y que el arduino imprime por el serial
    def leer_serial(self):
        if self.serial_port.in_waiting > 0:  # Si hay datos disponibles
            # datos = self.serial_port.readline().decode('utf-8').strip()
            try:

                datos = self.serial_port.read(self.serial_port.in_waiting).decode('utf-8').strip()
                self.Datos_serial.appendPlainText(datos)  # Agregar los datos en el espacio de datos serial
                # Condicionales para almacenar los datos de cada sensor en diferentes archivos txt, 
                # adicionando los datos cada que se reciban en lugar de sobreescribir en su propio txt
                if self.sensor_opcion == 'a':
                    with open("gps.txt", "a") as archivo:  # "a" para modo adición (append)
                        archivo.write(datos + "\n")
                if self.sensor_opcion == 'b':
                    with open("imu.txt", "a") as archivo: 
                        archivo.write(datos + "\n")
                if self.sensor_opcion == 'c':
                    with open("presion.txt", "a") as archivo: 
                        archivo.write(datos + "\n")
                if self.sensor_opcion == 'd':
                    with open("luz.txt", "a") as archivo:
                        archivo.write(datos + "\n")
                if self.sensor_opcion == 'e':
                    with open("temperatura.txt", "a") as archivo:  
                        archivo.write(datos + "\n")
                if self.sensor_opcion == 'f':
                    with open("voltaje.txt", "a") as archivo: 
                        archivo.write(datos + "\n")
                if self.sensor_opcion == 'g':
                    with open("corriente.txt", "a") as archivo:
                        archivo.write(datos + "\n")

                # Condicional para hacer graficas si el sensor seleccionado activo la opcion de grafica 
                if self.grafica == True:
                    # Dividir los datos en líneas
                    lineas = datos.splitlines()
                    # Inicializar una lista para los matches
                    matches = []
                    # Extraer números de cada línea ya sean enteros o decimales
                    for datos in lineas:
                        matches = re.findall(r'[-+]?\d*\.\d+|\d+', datos)  # Agregar todos los matches
                    # Solo se actualiza la grafica si se encuentran al menos un numero
                    if len(matches) >= 1:
                        valor = float(matches[0]) 
                        self.actualizar_grafica(valor)
                return datos
            except UnicodeDecodeError as e:
                # Maneja el error de decodificacion sin cerrar el programa
                print(f"Error de decodificacion: {e}")
                datos = ""
                return datos # Opcional: retorna una cadena vacia o realiza otro tipo de manejo
                

    # Función para actualizar la gráfica
    def actualizar_grafica(self, y):
        self.tiempo.append(time.time())  # Agregar la marca de tiempo actual
        self.valores.append(y)  # Agregar el valor del sensor
        # Limpiar la figura y redibujar la gráfica
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        ax.plot(self.tiempo, self.valores, 'b-')
        ax.set_title("Monitoreo")
        ax.set_xlabel("Tiempo")
        ax.set_ylabel("Valor")
         # Establecer límites de los ejes
        ax.set_xlim(min(self.tiempo) - 1, max(self.tiempo) + 1)  # Establece el límite del eje X
        ax.set_ylim(min(self.valores) - 1, max(self.valores) + 1)  # Establece el límite del eje Y
        # Ajustar la escala de tiempo
        ax.relim()
        ax.autoscale_view()
        self.canvas.draw()
        
    # Cerrar el puerto serial cuando se cierre la aplicación
    def closeEvent(self, event):
        self.serial_port.close()
        event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
