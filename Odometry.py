import math

i = 0    
L_data = []
R_data = []

def envia(ser, msg, t, ret):
    global i, L_data, R_data
    if msg.split(" ")[0] == 'GetMotors':
        r_msg = "a\nb\nc\nd\na,"+L_data[i]+"\na\nb\nc\nd,"+R_data[i]+"\nh,d\n"
        #print r_msg
        i = i + 1
        return r_msg

class odometry:
    def __init__(self, ser, start_point):
        self.S = 121.5
        self.ser = ser
        self.start_point = start_point
        self.theta = 0
        self.x = 0
        self.y = 0

        self.read_initial_wheel_position()

    def update(self):
        self.read_wheels()

    def read_wheels(self):
        """ Llegeix els valors L i R que retorna el robot i els actualitza. """

        resu = envia(self.ser, 'GetMotors LeftWheel RightWheel', 0.1, True).split("\n")

        L = float(resu[4].split(',')[1])
        R = float(resu[8].split(',')[1])

        print("L:", L, "R:", R)

        delta_l = L - self.Lprev
        delta_r = R - self.Rprev
        # print "L:", L, "R:",R, "L2:",L2, "R2: ",R2

        # Calcula la nova posicio (x,y)
        self.position_integration(delta_l, delta_r)
        # print "X:", x, "Y:", y

        self.Lprev = L
        self.Rprev = R
        # print "Theta",theta

    def updateTheta(self, delta_theta):
        self.theta = self.theta + delta_theta

        if self.theta > 2*math.pi:
            self.theta = self.theta - 2*math.pi

        if self.theta < 0:
            self.theta = self.theta + 2*math.pi

    def position_integration(self, R, L):
        """ Calcula la nova posicio de l'espai a partir del nous valors R i L. """
        dT = (R - L) / (2 * self.S)
        
        self.updateTheta(dT)

        dP = (R + L) / 2

        dx = dP * math.cos(self.theta)
        dy = dP * math.sin(self.theta)
        print "dx:",dx, "dy:", dy

        self.x = self.x + dx
        self.y = self.y - dy
        

    def read_initial_wheel_position(self):
        """ Inicializa los valores de la odometria para la localizacion del robot en el espacio. """
        
        resu = envia(self.ser, 'GetMotors LeftWheel RightWheel', 0.1, True).split("\n")
        
        self.Lprev = int(resu[4].split(',')[1])
        self.Rprev = int(resu[8].split(',')[1])

    def get_local(self):
        return (self.x, self.y)

    def get_world():
        return (self.x + self.start_point[0], self.y + self.start_point[1])

def main():
    global L_data,R_data,i

    L_data = [line.rstrip('\n') for line in open('L.txt')]
    R_data = [line.rstrip('\n') for line in open('R.txt')]

    odo = odometry(None, (0,0))

    for j in range(0,len(R_data)-1):
        odo.update()
        print "Local:", odo.get_local()

if __name__ == "__main__":
    main()
