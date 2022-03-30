from koopman.koopman import AEEncoder, KoopmanModel, KoopmanOperator
from koopman.trainer import Trainer
from sklearn.preprocessing import MinMaxScaler
import json
import numpy as np
import pickle
import torch

model_path = f"trained_models/sindy_model.pickle"

def get_sindy_representation():
    """ Function that can be called from Matlab script to get Koopman operators 
        and lifting from Python library. """

    ############################################################################
    ### 0. get measurements from json file 
    ############################################################################

    json_file = open("tmp/measurements.json")
    vars = json.load(json_file)
    json_file.close()

    y0 = np.array(vars["y0"])
    d0 = np.array(vars["d0"])

    # Adding batch dimension
    y0 = y0.reshape(1, -1)
    d0 = d0.reshape(1, -1)

    ############################################################################
    ### 1. Load model
    ############################################################################

    n_x, n_d, n_u = 5, 2, 1
    n_y = 1
    
    with open(model_path, 'rb') as f:
        sindy_model = pickle.load(f)


    state_scaler, input_scaler = MinMaxScaler(), MinMaxScaler()
    state_scaler.__dict__, input_scaler.__dict__ = sindy_model.state_scaler_params, sindy_model.input_scaler_params

    y0 = state_scaler(y0)
    y0 = state_scaler(y0)

    # Get operators 
    A, B = sindy_model._get_discrete_model()
    n_z = A.shape[0]
    C = np.zeros((n_y, n_z))
    C[0, 0] = 1

    def sindy_predictor(x0, u, n_steps):
        
        x0_scaled = state_scaler.transform(x0.reshape(1,-1))
        u_scaled = input_scaler.transform(u)
        if len(u_scaled.shape) == 1:
            u_scaled = u_scaled.reshape(-1,1)
        x_hat_scaled = np.zeros((n_steps+1, x0_scaled.shape[1]))
        x_hat_scaled[:,0] = sindy_model.predict(x0_scaled, u_scaled, n_steps, mode="d").reshape(-1)
        x_hat = state_scaler.inverse_transform(x_hat_scaled)
        return x_hat[:,0]

    ############################################################################
    ### 2. Lift data & get values
    ############################################################################

    z0, _ = ae_model.encoder.forward(y0, d0)
    z0 = z0.detach().numpy()

    A = ae_model.koopman_operator.A.weight.detach().numpy()
    B_u = ae_model.koopman_operator.B_u.weight.detach().numpy()
    B_d = ae_model.koopman_operator.B_d.weight.detach().numpy()
    C = ae_model.koopman_operator.C.weight.detach().numpy()

    ############################################################################
    ### 3. Save values
    ############################################################################
    
    data = {
        'z0': z0.tolist(),
        'A': A.tolist(),
        'B_u': B_u.tolist(),
        'B_d': B_d.tolist(),
        'C': C.tolist(),
    }
    
    json_string = json.dumps(data)

    with open('tmp/lifting.json', 'w') as outfile:
        outfile.write(json_string)


if __name__ == '__main__':
    get_sindy_representation()