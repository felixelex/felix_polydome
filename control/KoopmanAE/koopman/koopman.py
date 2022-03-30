import torch
from torch import Tensor
"""
This file contains all model classes.
"""

class KoopmanModel(torch.nn.Module):
    """ Koopman Auto-Encoder
    This module connects the different subcomponents (encoding layers, koopman 
    operator matrices).
    """
    def __init__(self, encoder, koopman_operator, n_y: int):
        """ Constructor method
        n_y is the output size, in our case 1 (temperature variable).
        """
        super().__init__()

        self.encoder = encoder
        self.koopman_operator = koopman_operator

        self.state_scaler = None
        self.input_scaler = None
        self.dist_scaler = None

        self.n_y = n_y
        self.is_autonomous = False

    def forward(self, x0: Tensor, d0: Tensor=None, 
                d: Tensor=None, u: Tensor=None, pred_steps: int=None):
        """ Encoding and prediction
        Method encodes a batch of samples and forwards them in feature space. 
        Number of time steps is defined by the second dimension of u.

        Inputs:
            x0:     [batch_size, time_delay+1, n_x]
            d0:     [batch_size, time_delay+1, n_d]
            d:      [batch_size, pred_steps, n_d]
            u:      [batch_size, pred_steps, n_u]
        Outputs:
            T_hat   [batch_size, pred_steps+1, 1]
            z       [batch_size, pred_steps+1, n_z]
        """
        batch_size = x0.size(0)
        if pred_steps == None:
            pred_steps = u.size(1)

        if u == None:
            self.is_autonomous = True

        # Encode
        x0 = x0.reshape(batch_size,-1)
        if d0 != None:
            d0 = d0.reshape(batch_size,-1)
            z0, _ = self.encoder(x0, d0)
        else:
            z0, _ = self.encoder(x0)

        print(z0)
        
        # Apply Koopman operator
        z = z0.reshape(z0.size(0), 1, z0.size(1))
        y = torch.empty((batch_size, 0, self.n_y))
        for i in range(pred_steps):
            if self.is_autonomous:
                z_next, y_i = self.koopman_operator(z[:,i,:])
            else:
                z_next, y_i = self.koopman_operator(z[:,i,:], d[:,i,:], u[:,i,:])
            z = torch.cat((z, z_next.view(z_next.size(0),1,z_next.size(1))), dim=1)
            y = torch.cat((y, y_i.view(y_i.size(0),1,y_i.size(1))), dim=1)

        # Calculate y_N
        if self.is_autonomous:
            _, y_N = self.koopman_operator(z[:,pred_steps,:])
        else:
            _, y_N = self.koopman_operator(z[:,pred_steps,:], torch.zeros_like(d[:,i,:]), torch.zeros_like(u[:,i,:]))
        y = torch.cat((y, y_N.view(y_N.size(0),1,y_N.size(1))), dim=1)

        print(y)
        print(z)

        T_hat = y
        return T_hat, z

class KoopmanOperator(torch.nn.Module):
    """ Submodule for Koopman operator
    """
    def __init__(self, n_z: int, n_d: int, n_u: int, n_y: int, bilinear_term: bool=False):
        super().__init__()

        self.is_autonomous = True if n_u == 0 else False

        self.A = torch.nn.Linear(n_z, n_z, bias=False)
        if n_u != 0:
            self.B_u = torch.nn.Linear(n_u, n_z, bias=False)
        if n_d != 0:
            self.B_d = torch.nn.Linear(n_d, n_z, bias=False)
        self.C = torch.nn.Linear(n_z, n_y, bias=False)
        if bilinear_term:
            self.B_uz = torch.nn.Linear(n_z, n_z, bias=False)

    def forward(self, z: Tensor, d: Tensor=None, u: Tensor=None):
        """ Forward dynamics one step in feature domain. """
        if hasattr(self, 'B_uz'):
            z_next = self.A(z) + self.B_u(u) + self.B_d(d) + self.B_uz(z * u)
            y = self.C(z)
        else:
            if self.is_autonomous:
                z_next = self.A(z)
                y = self.C(z)
            else:
                z_next = self.A(z) + self.B_u(u) + self.B_d(d)
                y = self.C(z)
        return z_next, y


class AECoder(torch.nn.Module):
    """ Abstract class that defines the way MLPs are constructed. Can be used
    for encoders and decoders. """
    def __init__(self, layer_arch: list, activation, dropout: float):
        """
        layer_arch: list
            Contains the input and output sizes of each layer. len(layer_arch)-1
            gives the total number of layers. No activation after the last 
            layer.
        """
        super().__init__()
        self.n_layers = len(layer_arch)-1
        self.layers = torch.nn.Sequential()

        for i in range(self.n_layers):
            # Linear layer
            self.layers.add_module(
                "linear{}".format(i), torch.nn.Linear(layer_arch[i], layer_arch[i+1])
            )

            # Activation functions & dropouts
            if (i!=self.n_layers-1): #don't place activaion after last layer
                self.layers.add_module(
                    "activation{}".format(i), activation
                )
                if (dropout is not None) and (dropout != 0):
                    self.layers.add_module(
                        "dropout{}".format(i), torch.nn.Dropout(dropout)
                    )

    def forward(self):
        raise NotImplementedError()
        

class AEEncoder(AECoder):
    def __init__(self, layer_arch: list, activation, dropout: float):
        super().__init__(layer_arch, activation, dropout)

    def forward(self, x0: Tensor, d0: Tensor=None):
        """
        Inputs:
            x0: [batch_size, n_x]
            d0: [batch_size, n_d]
        Outputs:
            z0: [batch_size, n_z]
        """
        if d0 != None:
            input = torch.cat((x0,d0), dim=1)
        else:
            input = x0
        z0 = self.layers(input)
        return z0, None


class VAEEncoder(AECoder):
    def __init__(self, layer_arch: list, activation, dropout: float):
        super().__init__(layer_arch, activation, dropout)

    def forward(self, x0: Tensor, d0: Tensor):
        """
        Model dimensions:
            encoder:    [in:    n_x + n_d, 
                         out:   2*n_z]
        Inputs:
            x0:      [batch_size, n_x]
            d0:      [batch_size, n_d]
        Outputs:
            z0:      [batch_size, n_z]
        """
        input = torch.cat((x0,d0), dim=1)
        param_f = self.layers(input)

        mu_f, logvar_f = param_f.split(param_f.size(1)//2, 1)

        if self.training:
            std_f = torch.exp(0.5 * logvar_f)
            z0 = torch.empty_like(mu_f).normal_() * std_f + mu_f
        else:
            # Use mean during evaluation
            z0 = mu_f

        return z0, param_f