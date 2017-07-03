""" Neato main module. The mission starts here! """
#!/usr/bin/python
# coding: utf8
import time
import math
import numpy as np
import serial
import mapper as mapper
import sys
import copy


# 175 pixels
# 8,85 metros
#mmPerPixel = 50.571428571
mmPerPixel = 20


# En milimetros
punto_A = (2520 + (172 * mmPerPixel), 1350 + (140 * mmPerPixel))
#punto_A = (2520 + (0 * mmPerPixel), 1350 + (0* mmPerPixel))
start_mm = punto_A
START = 3
END = 4

S = 121.5		# en mm
distancia_L = 0  # en mm
distancia_R = 0  # en mm
speed = 0 		# en mm/s
tita_dot = 0
tiempo = 5
direccion = 0


theta = 0
x = punto_A[0]
y = punto_A[1]
Pose_t = 0
Lprev = 0
Rprev = 0


def envia(ser, msg, t, ret):

    global rbuffer
    rbuffer = ''
    resp = ''

    ser.flushInput()
    ser.write(msg + chr(10))
    time.sleep(t)

    if ret:
        while ser.inWaiting() > 0:
            resp = ser.readline()
            rbuffer = rbuffer + resp
    # resp = ser.read(8192)
    # rbuffer = rbuffer + resp

    return rbuffer


def initSerial():
    error = 0

    try:
        ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
        ser.flushOutput()
        print '## Communication Process: Created Port Serial'

    except serial.SerialException:
        print '## Communication Process: Error!!!! Port Serial'
        error = 1
        ser = 0

    return ser, error


def poseIntegration(R, L):
    """ Calcula la nova posicio de l'espai a partir del nous valors R i L. """
    global theta, x, y, S, Pose_t

    dT = (R - L) / (2 * S)
    theta = theta + dT
    dP = (R + L) / 2

    dx = dP * math.cos(theta)
    dy = dP * math.sin(theta)

    x = dx + x
    y = y - dy

    Pose_t = np.matrix([x, y, theta])


def readPoseIntegration(ser):
    """ Llegeix els valors L i R que retorna el robot i els actualitza. """
    global Lprev, Rprev

    resu = envia(ser, 'GetMotors LeftWheel RightWheel', 0.1, True).split("\n")

    #print ("L:",L, "R:", R)
    L = int(resu[4].split(',')[1])
    R = int(resu[8].split(',')[1])

    L2 = L - Lprev
    R2 = R - Rprev
    # print "L:", L, "R:",R, "L2:",L2, "R2: ",R2

    # Calcula la nova posicio (x,y)
    poseIntegration(L2, R2)
    # print "X:", x, "Y:", y

    Lprev = L
    Rprev = R
    # print "Theta",theta


def init_robot(ser):
    """ Enviamos comadon al robot para que se ponga en modo test. """

    envia(ser, 'TestMode On', 0.2, False)
    print '## Communication Process: Neato TestMode On.'

    envia(ser, 'SetMotor RWheelEnable LWheelEnable', 0.3, False)

    # Enviamos comando al robot para que encender laser.
    envia(ser, 'SetLDSRotation On', 0.2, False)
    time.sleep(3)
    print '## Communication Process: Laser On.'

    # Enviamos comando al robot para que suene.
    envia(ser, 'PlaySound 1', 0.2, False)

    print '## Communication Process: Started.'


def adelante(ser, dist, speed):
    """ Hace avanzar al robot esa distancia a esa velocidad. """
    envia(ser, 'SetMotor LWheelDist ' + str(dist) + ' RWheelDist ' +
          str(dist) + ' Speed ' + str(speed), abs(dist) / speed + 0.1, False)


def giro_rad(ser, angulo, speed):
    """ Hace girar al robot el angulo especificado a esa velocidad. """
    comando = 'SetMotor LWheelDist ' + \
        str(-angulo * S) + ' RWheelDist ' + \
        str(angulo * S) + ' Speed ' + str(speed)
    envia(ser, comando, abs(angulo) * 500 / speed / math.pi, False)


def init_odometry(ser):
    """Inicializa los valores de la odometria para la localizacion del robot en el espacio."""
    global Lprev, Rprev
    resu = envia(ser, 'GetMotors LeftWheel RightWheel', 0.1, True).split("\n")

    #print ("L:",L, "R:", R)
    # Inicialitzem els valors L i R de les rodes amb els primers que ens retorna el robot abans de comencar a mourens.
    
    Lprev = int(resu[4].split(',')[1])
    Rprev = int(resu[8].split(',')[1])


def simplifica_camino(camino):
    """ Funcio que donat un conjunt de punts a l'espai, elimina tots els que no aporten cap canvi de direccio. """
    if (len(camino) == 0):
        print "Camino vacio"
        return []

    # Comencem amb el primer punt
    nuevo_camino = [camino[0]]
    prev_point = camino[0]

    # Guardem l'ultim punt del cami original, per si es descartat per l'algorisme, ja que podria estar en linia recta amb un altre.
    last_point = camino[len(camino) - 1]

    acum_giro = 0

    while camino:
        next_point = camino.pop(0)

        angu = mapper.cuantoGiro(prev_point, acum_giro, next_point)
        acum_giro += angu

        if abs(angu) > 0.001:
            nuevo_camino.append(next_point)

        prev_point = next_point

    if not last_point in nuevo_camino:
        nuevo_camino.append(last_point)

    return nuevo_camino

# Main


def point_to_json(point):
    return '{ "x": ' + str(point[1]) + ', "y": ' + str(point[0]) + '}'

def write_laser(laser_points, filename):
    """ Escribe los puntos del laser en un fichero de texto como jSON. """

    json_string = '{\n "laserPoints": ['
    for point in laser_points:
        json_string += point_to_json(point) + ',\n'
    json_string = json_string[:-2]
    json_string += ']\n}'

    with open(filename, 'w') as file_camino:
        file_camino.write(json_string)


def write_path(camino, filename):
    """ Escribe el camino en un fichero de texto  .Como jSON ? """

    json_string = '{\n "path": ['
    for point in camino:
        json_string += point_to_json(point) + ',\n'
    json_string = json_string[:-2]
    json_string += ']\n}'

    with open(filename, 'w') as file_camino:
        file_camino.write(json_string)


def map_laser(ser, mapa):
    global x,y
    """ Funcio encarregada de llegir el laser i posar les dades al mapa."""

    # Demanem les dades del laser al robot.
    msg_laser = envia(ser, "GetLDSScan", 0.1, True)
    angulo = 0

    laser_points = []

    # Per cada dada de laser llegida la tractem
    for line in msg_laser.split('\r\n')[2:362]:
        value = int(line.split(',')[1])
        # Si la distancia es superior a 4m descartem la dada.
        if value >= 4000:
            value = 0

        # Si la dada es una lectura valida.
        if value != 0:
            # Convertim la dada a cordenades x,y tenin en compte l'angle del robot
            x_l = value * math.cos(math.radians(angulo - math.degrees(theta)))
            y_l = value * math.sin(math.radians(angulo - math.degrees(theta)))

            # Afegim la posicio de la odometria a la posicio del laser
            laser_point = (x_l + x, y_l + y)
            print("Obstaculo:", laser_point)

            # Marquem l'obtacle al mapa
            mapper.markLaser(mapa, laser_point, 2)
            laser_points.append(laser_point)
        
        # Incrementem l'angle en un grau per tractar la seguent lectura.
        angulo += 1
    return laser_points

def save_map_as_image(mapa, filename):
    pixels = mapper.mapToPixels(mapa)
    mapper.savePixels(pixels, filename)


def go_for_it(ser, orig_in_mm, dest_in_mm, original_map, camino, simula):

    # Inicialitzem el punt previ com el punt origen on es troba el robot.
    prev_point = (orig_in_mm[0] + (172 * mmPerPixel), orig_in_mm[1] + (140 * mmPerPixel))
    # Suposem que el robot esta girat 0 graus.
    acum_giro = 0

    # Mentres quedin punts per anar al cami.
    while camino:

        #mapa = copy.deepcopy(original_map)
        next_point = mapper.convertToPoint(camino.pop())

        print("Go to:", mapper.convertToPixel(next_point))

        # Calculem quant ha de avanzar i girar el robot, per anar al seguent punt.
        dist = mapper.cuantoAvanzo(prev_point, acum_giro, next_point)
        angu = mapper.cuantoGiro(prev_point, acum_giro, next_point)

        print("Dist:", dist, " - Angu: ", angu)

        # Els afegin al nostre sistema de localitzacio independent de la odometria.
        prev_point = next_point
        acum_giro += angu

        # Si no estem en mode simulacio, efectivament mourem el robot.
        if simula == 0:
            # Fa gira el robot l'angle especificat a 75mm/s.
            giro_rad(ser, angu, 75)
            # Fa avancar el robot la distancia especificada a 150mm/s.
            adelante(ser, dist, 150)

            # Fem la Pose Integration per actualitzar la odometria.
            readPoseIntegration(ser)
            print("Estoy en: (" + str(x + punto_A[0]) + "," + str(y + punto_A[1]) + ")", "Theta:", theta)

        # time.sleep(1)

def ruta_demo(ser, mapa, orig_in_mm, dest_in_mm, simula):

    # Es marca al mapa els punts d'inici i final.
    mapper.markPoint(mapa, orig_in_mm, START)
    mapper.markPoint(mapa, dest_in_mm, END)
    #  184x140

    # Es preparen els punts, per a pasarlos al path-planning
    offset_x = (172*mmPerPixel)
    offset_y = (140*mmPerPixel)

    orig_int = (int(orig_in_mm[0] + offset_x), int(orig_in_mm[1] + offset_y))
    dest_int = (int(dest_in_mm[0] + offset_x), int(dest_in_mm[1] + offset_y))

    # Es busca el cami.
    b_camino, camino = mapper.FindPath(mapa, orig_int, dest_int)
    if b_camino:
        print "Voy a simplificar..."
        camino = simplifica_camino(camino)
        print("Camino simplificado:", camino)
    else:
        print("No hay camino", camino)
        
    
    # S'escriu el cami en un fitxer jSON, per a que el viewer el pugui mostrar.
    write_path(camino, "camino.json")
    #save_map_as_image(mapa, "salida.png")

    # Pasem el cami a la funcio que fara que es mogui el robot.
    go_for_it(ser, orig_in_mm, dest_in_mm, mapa, camino, simula)

def usage():
    if len(sys.argv) != 6:
        print "Usage: python", sys.argv[0], "<simula> <orig_in_mm> <dest_in_mm>"
        sys.exit(0)
    else:
        try:
            orig_x = float(sys.argv[2])
            orig_y = float(sys.argv[3])

            dest_x = float(sys.argv[4])
            dest_y = float(sys.argv[5])
            simula = int(sys.argv[1])

        except ValueError:
            print "Entrada incorrecta."
            sys.exit(0)        

        if int(sys.argv[1]) == 1:
            print "Simulando Ruta - De:", str(orig_x) + "," + str(orig_y), "==>", str(dest_x) + "," + str(dest_y)
        else:
            print "New Mission - De:", str(orig_x) + "," + str(orig_y), "==>", str(dest_x) + "," + str(dest_y)

        return (orig_x, orig_y, dest_x, dest_y, simula)

def main():
    """ Main function. """

    coords = usage()

    orig = (coords[0], coords[1])
    dest = (coords[2], coords[3])
    simula = coords[4]

    # Parametros Robot.
    # print "## Start Communication Process."
    if (simula == 0):
        ser, error = initSerial()
        init_robot(ser)
        init_odometry(ser)
    else:
        ser = None

    pixels = mapper.load_image_in_pixels("plano_clase_m20jun2017_export5.png")
    original_map = mapper.pixels_to_map(pixels)
    
    ruta_demo(ser, original_map, orig, dest, simula)


# Llamada a la funcion main
if __name__ == '__main__':

    main()
