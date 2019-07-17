#!/usr/bin/env python
from matplotlib.pyplot import *
from matplotlib.patches import Ellipse
from modules.filter import *

def simulate():

    # Simulation setup
    S_true = 10100.0
    S_true_list = [S_true]
    S_list = [0.0]
    P_list = [1e6]
    direction_true = -1.0
    l_list = [ 10000 - (i * 1000) for i in range(0,10) ]
    l_detected_list = []
    v_true = 15.0 # m/s, no acceleration
    delta_T = 1e-2
    iter = 0
    last_l = None
    detection_range = 15.0
    f = Filter()

    # Init
    while S_true > 0.0:
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
                will_detect = random.choice([1, 0], 1, p=[0.01, 0.99])
                if(not will_detect):
                    break
                l_true = l
                l_detected_list.append(l)
                omega_true = abs(dist)
                break 

        # Correct
        updated = False
        if(l_true is not None):
            noise_l = 0.0 # TODO: add landmark noise model to simulation
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

        # Plot
        if(iter % 5000 == 0 or updated):
            clf()
            ax = gca()
            title("Iter {}".format(iter))
            ylim(-0.5,1)
            xlabel("Position (in meters)")
            ylabel("Variance (in squared meters)")
            scatter(l_detected_list, zeros((len(l_detected_list))), c="yellow", s=300,marker="*", label="Detected Landmarks")
            plot(S_list, P_list, c="red", marker='x', label="Predicted")
            plot(S_true_list, zeros((len(S_true_list))), c="blue", marker='o', label="Groundtruth")

            for idx in range(len(S_list)):
                S = S_list[idx]
                S_true = S_true_list[idx]
                P = P_list[idx]

                # Error tag and line
                error = abs(S-S_true)
                x_mean = (0.8 * S + 0.2 * S_true)
                y_mean = (0.8 * P)
                #text(x_mean, y_mean, r'$\epsilon={}m$'.format(error))
                plot([S,S_true], [P,0.], c="k")

                # Confidence region
                ellipse = Ellipse(xy=(S, P), width=3 * np.sqrt(P), height=1e-2, 
                                edgecolor='k', fc='None', lw=1,)
                ax.add_patch(ellipse)

            legend()
            pause(0.1)
    show()
    return

if __name__=="__main__":
    simulate()