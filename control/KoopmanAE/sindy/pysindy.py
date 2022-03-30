import pysindy as ps
import pickle
import numpy as np
from sklearn.cluster import KMeans
from sklearn.preprocessing import MinMaxScaler
from scipy.signal import cont2discrete

################################################################################
### Notes to myself
################################################################################
"""
Using a scaler in the lifted domain leads to a poor fit and thus should be
avoided. Why I can't explain intuitivly.
Possible answer: Using RBF kernels, the lifted data is somewhat already scaled 
on the interval [0,1]. Some negligable features nevertheless might be upscaled 
so that they start looking non-negligable.
"""
################################################################################
###
################################################################################

class sindyModel():
    """ Wrapper class for PySINDY library """
    def __init__(self, 
        sindy_params=None, 
        sigma=1, 
        n_kernels=10, 
        id_lifting=None, 
        seed=0, 
        poly_order=0,
        discrete_time=True):
        
        self.input_scaler_params = None
        self.state_scaler_params = None

        self.sigma = sigma
        self.n_kernels = n_kernels
        self.fit_intercept = False
        self.id_lifting = id_lifting
        self.poly_order = poly_order
        self.discrete_time = discrete_time
        self.seed = seed
        self.sindy_params = {'threshold':0.01, 'alpha':10} if sindy_params is None else sindy_params
        self.sindy = ps.SINDy(
            optimizer = ps.STLSQ(**self.sindy_params, fit_intercept=self.fit_intercept),
            feature_library = ps.IdentityLibrary(),
            discrete_time = self.discrete_time
            )
        if self.poly_order > 0:
            self.polyLib = ps.PolynomialLibrary(
                    degree=self.poly_order,
                    include_interaction=True,
                    interaction_only=True,
                    include_bias=True
                    )
        else:
            self.polyLib = None
        
        self.discrete_model = None

    def fit(self, x_train, u_train=None):
        """ Fits the SindyModel to the dataset. """
        _ = self._get_kernel_centers(x_train)
        if self.poly_order > 0:
            self.polyLib.fit(x_train)

        if isinstance(x_train, list):
            x_lifted = [self.lift_data(x) for x in x_train]
        else:
            x_lifted = self.lift_data(x_train)

        self.is_autonomous = True if u_train is None else False

        if isinstance(x_train, list):
            self.n_x = x_train[0].shape[1]
        else:
            self.n_x = x_train.shape[1]
        self.n_z = self.n_kernels

        if self.is_autonomous:
            self.sindy.fit(x=x_lifted, quiet=True) 
            self.n_u = 0
        else:
            multiple_trajectories = True if isinstance(x_lifted, list) else False
            self.sindy.fit(x=x_lifted, u=u_train, quiet=True, multiple_trajectories=multiple_trajectories)

            if isinstance(u_train, list):
                self.n_u = u_train[0].shape[1]
            else:
                self.n_u = u_train.shape[1]


    def predict(self, x0, u, n_steps, mode="c"):
        """ Predict n_steps into future. 
        Modes:
            "c"     continues
            "d"     discrete (what will be used in MPC)
        """
        x0 = x0.reshape(1, -1)
        x0 = self.lift_data(x0)
        x0 = x0.reshape(1, -1)

        if mode=="c":
            if self.is_autonomous:
                x_hat = self.sindy.simulate(x0=x0[0], t=np.arange(0,n_steps))
            else:
                if u.shape[0] != n_steps:
                    u = u[:n_steps]
                t = np.arange(0,n_steps) if self.discrete_time is False else n_steps
                x_hat = self.sindy.simulate(x0=x0[0], t=t, u=u)
        elif mode=="d":
            A, B  = self._get_discrete_model()
            C = np.zeros((1, A.shape[1]))
            C[:,0] = 1
            D = np.zeros((1, B.shape[1]))
            sys = (A, B, C, D)
            x_hat = self._discrete_time_predictions(sys, x0, u, n_steps)
        return x_hat

    def _discrete_time_predictions(self, sys, x0, u, n_steps):
        A, B, C, D = sys
        y = np.empty((n_steps+1, C.shape[0]))
        y[0] = np.matmul(C,x0.T)
        x = x0.T
        u = u[:n_steps]
        for i in range(n_steps):
            x = np.matmul(A, x) + np.matmul(B,u[i].reshape(-1,1))
            y[i+1] = np.matmul(C, x) + np.matmul(D,u[i].T)
        return y

    def print(self):
        """ Print dynamic equations """
        self.sindy.print()

    def get_Koopman_operator(self):
        """ Get A & B matrices. """
        A = self.sindy.coefficients()[:,:-self.n_u]
        B = self.sindy.coefficients()[:,-self.n_u:]
        return A, B

    def lift_data(self, x):
        """ Lifts a numpy data set. """
        if self.id_lifting is not None:
            x_id = x[:,self.id_lifting]
        n_samples = x.shape[0]

        x_lifted = np.empty((n_samples,0))

        # RBF
        if self.n_kernels != 0:
            x_c = np.expand_dims(x, axis=2)
            x_c = np.repeat(x_c, self.n_kernels, axis=2)

            c = self.centers.T
            c = np.expand_dims(c, axis=0)
            c = np.repeat(c, n_samples, axis=0)

            norm = np.power((x_c-c), 2).sum(axis=1)

            x_lifted_rbf = np.exp(-norm/(2*self.sigma**2))
            x_lifted = np.hstack((x_lifted, x_lifted_rbf))

        # Polynomial
        if self.poly_order != 0:
            x_lifted_poly = self.polyLib.transform(x)
            x_lifted = np.hstack((x_lifted, x_lifted_poly))            

        # Identity
        if self.id_lifting is not None:
            x_lifted = np.hstack((x_id, x_lifted))
        return x_lifted

    def _get_kernel_centers(self, x_train):
        """ Finds kernel positons via K-Means. """
        if isinstance(x_train, list):
            x_train = np.vstack(x_train)

        kmeans = KMeans(n_clusters=self.n_kernels, random_state=self.seed).fit(x_train)
        self.centers = kmeans.cluster_centers_
        return kmeans.cluster_centers_

    def _get_Koopman_operators(self):
        """ Returns the Koopman operators A,B. The returned matrices represent 
        the discrete/continues model, depending on the settings during id. """

        n_u = self.sindy.n_control_features_
        A = self.sindy.coefficients()[:,:-n_u]
        B = self.sindy.coefficients()[:,-n_u:]
        return A, B

    def _get_discrete_model(self, dt=1):
        """ Returns the discrete-time model. This has been either directly 
        identified or is a zoh-version of the continues-time model. """
        if self.discrete_model is not None:
            return self.discrete_model
        else:
            A, B = self._get_Koopman_operators()
            is_discrete = self.discrete_time
            if is_discrete:
                self.discrete_model = (A, B)
                return A, B
            else:
                C = np.zeros((1, A.shape[1]))
                D = np.zeros((1, B.shape[1]))
                disc_system = cont2discrete((A, B, C, D), dt=dt, method="bilinear")
                A, B = disc_system[0], disc_system[1]
                return A, B
