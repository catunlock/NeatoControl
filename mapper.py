from PIL import Image
import numpy as np
import heapq as pq
import math

WIDTH = 0
HEIGHT = 0
START = 3
END = 4

def load_image_in_pixels(filepath):
    """ Carrega la imatge i la retorna com una matriu de pixels RGB. """
    
    global WIDTH, HEIGHT
    
    img = Image.open(filepath)
    rgb_im = img.convert('RGB')

    # Obtenim una llista de pixels
    pixels = list(rgb_im.getdata())

    # Obtenim la mida de la imatge
    WIDTH, HEIGHT = rgb_im.size
   
    # Convertirm aquesta llista de pixels en una matriu.
    pixels = [pixels[i * WIDTH:(i + 1) * WIDTH] for i in xrange(HEIGHT)]

    return pixels

def pixels_to_map(pixels):
    mapa = np.zeros((len(pixels),len(pixels[0])))

    for i in range(0, len(mapa) - 1 ):
        for j in range(0, len(mapa[i]) - 1):
            if pixels[i][j] == (255, 255, 255):
                mapa[i][j] = 0
            elif pixels[i][j] == (128,128,128):
                mapa[i][j] = 6
            else:
                mapa[i][j] = 1
    
    return mapa

def mapToPixels(mapa):
    pixels = np.zeros((len(mapa),len(mapa[0]),3), 'uint8')

    
    for i in range(0,  len(mapa) - 1):
        for j in range(0, len(mapa[i]) - 1):
            if mapa[i][j] == 0:     # Vacio
                pixels[i][j][0] = 255
                pixels[i][j][1] = 255
                pixels[i][j][2] = 255
                
            elif mapa[i][j] == 2:   # Laser
                pixels[i][j][0] = 255
                pixels[i][j][1] = 0
                pixels[i][j][2] = 0

            elif mapa[i][j] == 3:   # Start
                pixels[i][j][0] = 0
                pixels[i][j][1] = 255
                pixels[i][j][2] = 0

            elif mapa[i][j] == 4:   # End
                pixels[i][j][0] = 0
                pixels[i][j][1] = 0
                pixels[i][j][2] = 255

            elif mapa[i][j] == 5:   # Path
                pixels[i][j][0] = 255
                pixels[i][j][1] = 255
                pixels[i][j][2] = 0
            elif mapa[i][j] == 6:
                pixels[i][j][0] = 128
                pixels[i][j][1] = 128
                pixels[i][j][2] = 128

            else:                   # Muro == 1
                pixels[i][j][0] = 0
                pixels[i][j][1] = 0
                pixels[i][j][2] = 0

    return pixels

def savePixels(pixels, filepath):
    c = np.asarray(pixels)
    img = Image.fromarray(c)
    img.save(filepath)

def get_path(mapa, predecesor, n):

    camino = []
    k = n
    if k not in predecesor:
        print("k no es en predecesor.")

    while k in predecesor:
        mapa[k[0]][k[1]] = 4
        camino.append(k)
        k = predecesor[k]
        
    
    return camino


def distManhattan(a,b):
    return abs(a[0] - b[0]) + abs(a[1]- b[1])

def FindPath(mapa, start, goal):
    """ Funcio per trobar un cami a un mapa, des de un punt inicial a un final. """

    start = convertToPixel(start)
    goal = convertToPixel(goal)
    
    print("Buscando:", start, "->", goal)
    queue = []
    visited = set()
    
    # Per veure a cada node des de quin altre s'ha arribat per poder tornar en enrere i obtenir el cami.
    predecesor = {}

    visited.add(start)
    pq.heappush(queue, (1,start))

    while queue:
        v = pq.heappop(queue)[1]

        # Per a tots els veins
        for p in [(-1,0),(1,0),(0,-1), (0,1)]:
            n = (v[0] + p[0], v[1] + p[1])
          
            c1 = n[0] < len(mapa) - 1
            c2 = n[1] < len(mapa[0]) - 1

            c3 = mapa[n[0]][n[1]] == 0 or mapa[n[0]][n[1]] == 4 

            # Si compleixen totes les condicions (navegable), el posem a la cua.
            if n not in visited and n[0] >= 0 and n[1] >= 0 and c1 and c2  and c3:
                mapa[n[0]][n[1]] = 5
                visited.add(n)
                pq.heappush(queue, (distManhattan(n, goal), n))
                predecesor[n] = v

            # Si es on voliem arribar, retornem el cami per on hem arribat.
            if n == goal:
                print "Goal"
                return True, get_path(mapa, predecesor, n)
    
    return False, []

#175 pixels
#8,85 metros
mmPerPixel = 20


# En milimetros
punto_A = (2520 + (10 * mmPerPixel), 1350 + (42* mmPerPixel))
start_mm = punto_A

def convertToPixel(point_mm):
    global mmPerPixel

    x = int((point_mm[0]) / mmPerPixel)
    y = int((point_mm[1]) / mmPerPixel)
    return (x,y) 

def convertToPoint(pixel):
    global mmPerPixel

    x = int((pixel[0] * mmPerPixel))
    y = int((pixel[1] * mmPerPixel))

    return (x,y)

def cuantoGiro(odometria, theta, destino):
    return math.atan2(destino[1] - odometria[1], destino[0] - odometria[0]) - theta

def cuantoAvanzo(odometria, theta, destino):
    return math.sqrt(math.pow(destino[0] - odometria[0], 2) + math.pow(destino[1] - odometria[1], 2)) 


def markPoint(mapa, point_mm, v):
    pixel = convertToPixel(point_mm)
    mapa[pixel[0]][pixel[1]] = v

def markLaser(mapa, point_mm, v):
    pixel = convertToPixel(point_mm)
    for i in [-1,0,1]:
        for j in [-1,0,1]:
            mapa[pixel[0]+ i][pixel[1]+ j] = v
    

def main():
    """ Main Function."""
    pixels = loadImageInPixels("map_20_bw_gris.png")



    #print(pixels)
    # 1, 6 -> blanco
    # 0, 6 -> negro
    # 1, 5 -> negro

    mapa = pixelsToMap(pixels)
    print(mapa)
    #print mapa


    dest = convertToPixel((5000,11000))
    orig = convertToPixel(punto_A)

    print("ORIG:", orig)
    print("DEST:", dest)

    markPoint(mapa, punto_A, START)
    markPoint(mapa, (5000,11000), END)

    newOrig = (int(orig[0]),int(orig[1]))
    newDest = (int(dest[0]), int(dest[1]))


    print("DEST:", mapa[newDest[0]][newDest[1]])

    if FindPath(mapa, newOrig, newDest):
        print("Hay camino")

    pixels = mapToPixels(mapa)

    savePixels(pixels, "salida.png")

    

if __name__ == "__main__":
    main()
