import sys
sys.path.insert(0,"/KoopmanAE/")

from sklearn.preprocessing import MinMaxScaler
import json
import numpy as np
import pickle
import torch

model_path = f"KoopmanAE/trained_models/sindy_model.pickle"

def get_sindy_representation():
    """ Function that can be called from Matlab script to get Koopman operators 
        and lifting from Python library. """

    ############################################################################
    ### 0. get measurements from json file 
    ############################################################################

    json_file = open("KoopmanAE/tmp/measurements.json")
    vars = json.load(json_file)
    json_file.close()

    x0 = np.array(vars["x0"])
    d0 = np.array(vars["d0"])

    # Adding batch dimension
    x0 = x0.reshape(1, -1)
    d0 = d0.reshape(1, -1)

    ############################################################################
    ### 1. Load model
    ############################################################################

    n_x, n_d, n_u = 4, 2, 1
    n_y = 1
    
    with open(model_path, 'rb') as f:
        sindy_model = pickle.load(f)


    state_scaler = MinMaxScaler()
    state_scaler.__dict__ = sindy_model.state_scaler_params

    x0 = state_scaler.transform(x0)

    ############################################################################
    ### 2. Lift data & get values
    ############################################################################

    z0 = sindy_model.lift_data(x0)

    A, B = sindy_model._get_discrete_model()
    n_z = A.shape[0]
    B_d = B[:,:n_d]
    B_u = B[:,n_d:]
    C = np.zeros((1, n_z))
    C[0,0] = 1

    ############################################################################
    ### 3. Save values
    ############################################################################
    
    data = {
        'z0': z0.tolist(),
        'A': A.tolist(),
        'B_u': B_u.tolist(),
        'B_d': B_d.tolist(),
        'C': C.tolist(),
        'T_min': state_scaler.min_[0],
        'T_scale': state_scaler.scale_[0],
    }
    
    json_string = json.dumps(data)

    with open('KoopmanAE/tmp/lifting.json', 'w') as outfile:
        outfile.write(json_string)

if __name__ == '__main__':
    get_sindy_representation()