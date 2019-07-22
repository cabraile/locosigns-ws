#!/usr/bin/env python
from matplotlib.pyplot import *
from matplotlib.patches import Ellipse
from modules.filter import *

from numpy import *

# ENVIRONMENT SETTINGS
SIGMA_L=(100/3.0)           # The variance in the placement of the landmarks
DELTA_T=1e-2                # The interval of time between each iteration
SIGMA_OMEGA=1e-1            # The variance in the sensed depth of the landmark
DETECTION_RATE=0.7          # The probability of detecting the landmark
MOVING_FORWARD=True        # Whether the robot is moving forward on the road (True) or backwards (False).
STARTING_POSITION=0.0       # The starting position of the robot in meters
N_LANDMARKS = 100           # Number of landmarks

# INTERFACE SETTINGS
PLOT_ELLIPSE=False          # Convidence ellipse around the estimation
PLOT_GT_LINE=False          # Line between the groundtruth and the estimation
PRINT_ERROR = True          # Print on the terminal the error between the estimation and the groundtruth
PLOT_FREQ = 5000            # Display the graph each PLOT_FREQ iterations
PLOT_ERROR = True           # Plot the error line
STORE_STATES_FREQ = 500     # Store states every STORE_STATES_FREQ iteration for plotting latter

def endCondition(x):
    if(MOVING_FORWARD):
        return x < 100000.0
    else:
        return x > 0.0

def simulate():
    # Simulation setup
    if(MOVING_FORWARD):
        S_true_init = 0.0
    else:
        S_true_init = 100100.0
    S_true = S_true_init
    S_true_list = [S_true]
    S_init = -50.
    S_list = [S_init]
    P_list = [1e6]
    error_list = array([S_init, (S_true_init - S_init) ** 2.0])
    # - Noisy landmark position
    l_true_list = None
    direction_true = None
    if MOVING_FORWARD:
        direction_true = 1.0
        l_true_list = array([ (i * 1000) for i in range(0,N_LANDMARKS) ])
    else:
        direction_true = -1.0
        l_true_list = array([ 1000 * N_LANDMARKS - (i * 1000) for i in range(0,N_LANDMARKS) ])
    l_noise = (2.0 * random.rand(l_true_list.shape[0]) - ones(l_true_list.shape) ) *\
            SIGMA_L * 3.0
    l_list = l_true_list + l_noise
    l_detected_list = []
    v_true = 15.0 # m/s, no acceleration
    delta_T = DELTA_T
    iter = 0
    last_l = None
    detection_range = 15.0
    f = Filter(delta_T,SIGMA_L, SIGMA_OMEGA)
    
    while endCondition(S_true):
        iter += 1
        # Prediction
        v_noise = (0.1 * v_true) * ( 2.0 * ( random.rand() - 0.5 ) )
        v = v_true + v_noise
        S_true += direction_true * v_true * delta_T
        f.predict(v)

        # Measurement
        l_true = None
        omega_true = 0.0
        psi_true = 0.0
        for l in l_list:
            dist = l - S_true
            if(dist <= detection_range) and (dist > 2.0):
                # Does not consider if the current landmark is the same as the previous - TODO add this measurement for correction
                if(last_l is not None and last_l == l):
                    continue
                # There is a chance that the landmark is not detected
                _p = DETECTION_RATE
                will_detect = random.choice([1, 0], 1, p=[_p, 1.0-_p])
                if(not will_detect):
                    break
                l_true = l
                l_detected_list.append(l)
                omega_true = abs(dist)
                break 

        # Correct
        updated = False
        if(l_true is not None):
            noise_omega = 3.0 * (1e-1) * ( 2.0 * ( random.rand() - 0.5 ) )
            noise_psi = 0.0 # TODO: add psi noise model to simulation
            omega = omega_true + noise_omega
            psi = psi_true + noise_psi
            updated = f.update(l_true, omega, psi)
            last_l = l

        if(iter % 500 == 0 or updated):
            S_list.append(f.S)
            P_list.append(f.P)
            S_true_list.append(S_true)
            error=array([S_true, (f.S - S_true) ** 2.0])
            error_list = vstack( (error_list, error ) )

        # Plot
        if(iter % 5000 == 0 or updated):
            clf()
            ax = gca()
            title("Iter {}".format(iter))
            xlim(0., 1000 * N_LANDMARKS)
            ylim(-0.5,50)
            xlabel("Position (in meters)")
            ylabel("Variance (in squared meters)")
            scatter(l_detected_list, zeros((len(l_detected_list))), c="yellow", s=300,marker="*", label="Detected Landmarks")
            plot(S_list, P_list, c="red", marker='x', label="Predicted")
            plot(S_true_list, zeros((len(S_true_list))), c="blue", marker='o', label="Groundtruth")

            # Error line
            if PLOT_ERROR:
                plot(error_list[:,0], error_list[:,1], c="green", label="Squared error")
            
            for idx in range(len(S_list)):
                S = S_list[idx]
                S_true = S_true_list[idx]
                P = P_list[idx]


                # Groundtruth line
                if PLOT_GT_LINE:
                    plot([S,S_true], [P,0.], c="k")

                # Confidence region
                if PLOT_ELLIPSE:
                    ellipse = Ellipse(xy=(S, P), width=P, height=P, 
                                    edgecolor='k', fc='None', lw=1,)
                    ax.add_patch(ellipse)

            legend()
            pause(0.1)
    show()
    return

if __name__=="__main__":
    simulate()