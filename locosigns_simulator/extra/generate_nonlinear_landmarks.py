from numpy import *
from matplotlib.pyplot import *

def flush(worldfile, x, y, idx):
    lines = [
    "<include>\n",
        "\t<uri>model://speed_limit_sign</uri>\n",
        "\t<name>landmark_{}</name>\n".format(idx),
        "\t<pose>{} {} 0 0 0 1.57079632679</pose>\n".format(x,y),
    "</include>\n"
    ]
    worldfile.writelines(lines)
    return

def genLandmarks(radius, alpha):
    landmarks = zeros((8,2))
    x_c_1 = radius
    x_c_2 = 3.0 * radius
    factor = radius * cos(alpha)
    # Landmark 1
    _x = x_c_1 - factor
    landmarks[0,0] = _x
    landmarks[0,1] = sqrt( -_x**2.0 + 2.0*radius *_x  )
    # Landmark 2
    landmarks[1,0] = radius
    landmarks[1,1] = radius
    # Landmark 3
    _x = x_c_1 + factor
    landmarks[2,0] = _x
    landmarks[2,1] = sqrt( -_x**2.0 + 2.0*radius *_x  ) 
    # Landmark 4
    landmarks[3,0] = 2.0 * radius
    landmarks[3,1] = 0.0
    # Landmark 5
    _x = x_c_2 - factor
    landmarks[4,0] = _x
    landmarks[4,1] = - sqrt( radius**2.0 - (_x - x_c_2) ** 2.)
    # Landmark 6
    landmarks[5,0] = 3.0 * radius
    landmarks[5,1] = -radius
    # Landmark 7
    _x = x_c_2 + factor
    landmarks[6,0] = _x
    landmarks[6,1] = - sqrt( radius**2.0 - (_x - x_c_2) ** 2.)
    # Landmark 2
    landmarks[7,0] = 4*radius
    landmarks[7,1] = 0
    return landmarks

def main():
    # Parameters
    R = 4000.0 / pi # radius of the road (in meters)
    alpha = pi/4.0 
    landmarks = genLandmarks(R, alpha)
    # Creates tracks and then flush them to the world file
    worldfile = open("landmarks.world","w")
    for i in range(landmarks.shape[0]):
        flush(worldfile, landmarks[i,1], landmarks[i,0]-5.0, i+1)
    worldfile.close()
    return

if __name__ == "__main__":
    main()