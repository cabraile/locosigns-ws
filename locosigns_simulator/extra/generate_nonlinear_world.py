from numpy import *

class RoadTrack():

    def __init__(self, startpoint, endpoint, idx):
        self.startpoint = startpoint
        self.endpoint = endpoint
        self.index = idx
        return
    
    def flush(self, worldfile):
        lines = [
            '<road name="road_{}">\n'.format(self.index),
                '\t<width>7.34</width>\n',
                '\t<point>{} {} 0</point>\n'.format(self.startpoint[1], self.startpoint[0]),
                '\t<point>{} {} 0</point>\n'.format(self.endpoint[1], self.endpoint[0]),
            '</road>\n'
        ]
        worldfile.writelines(lines)
        return

def genRoadPoints(radius, numbersamples):
    R = radius
    N = numbersamples
    # Generate both semi-circles
    X_sc_1 = linspace(0.0, 2.0 * R, N, endpoint=True)
    Y_sc_1 = sqrt( (8000.0/pi) * X_sc_1 - square(X_sc_1) ) # Y values for the first semi-circle
    X_sc_2 = linspace(2.0 * R, 4.0 * R, N, endpoint=True)
    Y_sc_2 = -sqrt( (4000.0/pi)**2 - square(X_sc_2 - (12000.0 / pi )) ) # Y values for the second semi-circle
    # Append points sequentially - important
    list_points = []
    for idx in range(0,N):
        list_points.append( array([X_sc_1[idx], Y_sc_1[idx]]) )
    for idx in range(0,N):
        list_points.append( array([X_sc_2[idx], Y_sc_2[idx]]) )
    return list_points

def main():
    # Parameters
    R = 4000.0 / pi # radius of the road (in meters)
    N = 1000 # number of samples for the discretization
    list_points = genRoadPoints(R,N)
    # Creates tracks and then flush them to the world file
    worldfile = open("world_file.world","w")
    for i in range(1, len(list_points)):
        startpoint = list_points[i-1]
        endpoint = list_points[i]
        track = RoadTrack(startpoint, endpoint, i-1)
        track.flush(worldfile)
    worldfile.close()
    return

if __name__ == "__main__":
    main()