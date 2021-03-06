import sys
sys.path.insert(0,"/KoopmanAE/koopman")

from sklearn.preprocessing import MinMaxScaler
import numpy as np
import json
import torch
from koopman.koopman import AEEncoder, KoopmanModel, KoopmanOperator
from koopman.trainer import Trainer

model_path = "KoopmanAE/trained_models/AE_td0_Polydome.pt"

def get_ae_representation():
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

    ############################################################################
    ### 1. Load model
    ############################################################################

    n_x, n_d, n_u = 4, 2, 1
    n_z, n_y = 6, 1
    time_delay = 0
    w = 16
    n_layers = 2

    hidden_layers = [w]*(n_layers-1)
    encoder_arch = [(time_delay+1)*(n_x+n_d), *hidden_layers, n_z]

    encoder = AEEncoder(
        layer_arch=encoder_arch, 
        activation=torch.nn.ReLU(),
        dropout=0
        )

    koopman_operator = KoopmanOperator(n_z, n_d, n_u, n_y)

    ae_model = KoopmanModel(
        encoder=encoder,
        koopman_operator=koopman_operator,
        n_y=n_y
    )

    trainer = Trainer()
    trainer.load(ae_model, model_path)

    ############################################################################
    ### 2. Lift data & get values
    ############################################################################

    state_scaler, dist_scaler = MinMaxScaler(), MinMaxScaler()

    state_scaler.__dict__ = ae_model.state_scaler
    dist_scaler.__dict__ = ae_model.dist_scaler

    # Adding batch dimension
    x0 = x0.reshape(1, -1)
    d0 = d0.reshape(1, -1)

    x0 = state_scaler.transform(x0)
    d0 = dist_scaler.transform(d0)

    x0 = torch.Tensor(x0)
    d0 = torch.Tensor(d0)

    z0, _ = ae_model.encoder.forward(x0, d0)
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
        'T_min': state_scaler.min_[0],
        'T_scale': state_scaler.scale_[0],
    }
    
    json_string = json.dumps(data)

    with open('KoopmanAE/tmp/lifting.json', 'w') as outfile:
        outfile.write(json_string)


if __name__ == '__main__':
    get_ae_representation()