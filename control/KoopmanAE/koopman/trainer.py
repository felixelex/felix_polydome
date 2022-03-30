import torch
from tqdm import tqdm
from sklearn.preprocessing import MinMaxScaler
import numpy as np

################################################################################
""" 
TODO:
"""
################################################################################

class Trainer():
    def __init__(self) -> None:
        self.is_autonomous = False
        pass

    def train(self, model, train_data, val_data, train_args):
        
        torch.manual_seed(train_args["seed"])

        # Optimizer, Criterion, LRScheduler
        self.optimizer = torch.optim.Adam(model.parameters(), train_args["initial_lr"])
        self.criterion = torch.nn.MSELoss(reduction="mean")
        self.lr_scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
            self.optimizer,
            patience = train_args["patience"],
            factor = 0.3,
            min_lr = 1e-7,
            cooldown = 1000
        )

        # Load data into torch
        x_train = torch.from_numpy(train_data['x']).to(dtype=torch.float)
        x_val = torch.from_numpy(val_data['x']).to(dtype=torch.float)
        if train_data['u'] is not None:
            self.is_autonomous = False
            u_train = torch.from_numpy(train_data['u']).to(dtype=torch.float)
            u_val = torch.from_numpy(val_data['u']).to(dtype=torch.float)
        else:
            self.is_autonomous = True
        if train_data['d'] is not None:
            d_train = torch.from_numpy(train_data['d']).to(dtype=torch.float)
            d_val = torch.from_numpy(val_data['d']).to(dtype=torch.float)

        # Get weights
        c1 = train_args.get("c1", 0)    #c1 0 for AE
        c2 = train_args["c2"]
        c3 = 0
        c4 = 0
        c5 = 0

        time_delay = train_args["encoding_time_delay"]

        ########################################################################
        ## TRAINING
        ########################################################################
        pbar = tqdm(range(train_args["max_iter"])) if train_args["verbose"] else range(train_args["max_iter"])
        for i in pbar:

            model.train()

            # Warm-start encoder-decoder
            if i == train_args["iter_enc_dec"]:
                c3 = train_args["c3"]
                c4 = train_args["c4"]
                c5 = train_args["c5"]

            index = self._create_rand_index(
                train_args["batch_size"], 
                train_args["train_pred_steps"] + time_delay, 
                x_train.size(0))


            # x_batch = x_train[index, :] #x: [batch_size, pred_steps+time_delay+1, n_x]
            if len(x_train.shape) > 2:
                index = index + (index[:,0]*x_train.size(1)).unsqueeze(1).repeat(1, index.size(1))
                x_batch = x_train.reshape(-1, x_train.size(2))[index, :]
            else:
                x_batch = x_train[index, :] #x: [batch_size, pred_steps+time_delay+1, n_x]
            
            if self.is_autonomous == False:
                u_batch = u_train[index, :] #u: [batch_size, pred_steps+time_delay+1, n_u]  
                d_batch = d_train[index, :] #d: [batch_size, pred_steps+time_delay+1, n_d]

                u_batch = u_batch[:,:train_args["train_pred_steps"]+time_delay,:] #u: [batch_size, pred_steps+time_delay, n_u]

            # x:  0 1 2 ... k-1 k       -> length = k+1
            # d:  0 1 2 ... k-1 k       -> length = k+1
            # u:  0 1 2 ... k-1         -> length = k

            # Zero gradient
            self.optimizer.zero_grad(set_to_none=True)

            # Forward
            x0 = x_batch[:,0:time_delay+1,:]
            if self.is_autonomous == False:
                d0 = d_batch[:,0:time_delay+1,:]
                d_batch = d_batch[:,time_delay+1:,:]
                u_batch = u_batch[:,time_delay:,:]

                T_hat, z_hat = model.forward(x0=x0, d0=d0, d=d_batch, u=u_batch)
                """ Dimensions:
                T_hat   [batch_size, pred_steps+1, 1]
                z_hat   [batch_size, pred_steps+1, n_z]
                """
            else:
                T_hat, z_hat = model.forward(x0, pred_steps=train_args["train_pred_steps"])

            print(x0.shape)
            print(d0.shape)
            print(d_batch.shape)
            print(u_batch.shape)
            print(self.is_autonomous)
            print(T_hat)
            assert False

            # Loss
            l1, l2, l3, l4, l5 = 0, 0, 0, 0, 0
            # L1: KL-Loss
            if c1 != 0:
                _, param_f = model.encoder.forward(x0.reshape(x0.size(0),-1), d0.reshape(x0.size(0),-1))
                mu_f, logvar_f = param_f.split(param_f.size(1)//2, 1)
                kl = - 0.5 * (1 + logvar_f - mu_f.pow(2) - logvar_f.exp())
                l1 = kl.sum() / d_batch.size(0)
            # L2: Autoencoder loss
            if c2 != 0:
                if self.is_autonomous: #THIS ONLY WORKS IN MY USE CASE AND IS NOT CORRECT IN GENERAL
                    l2 = self.criterion(T_hat[:,0,:], x0[:,time_delay,:]) # T only
                else:
                    l2 = self.criterion(T_hat[:,0,:], x0[:,time_delay,0:1]) # T only
            # L4: Prediction loss
            if c4 != 0:
                if self.is_autonomous: #THIS ONLY WORKS IN MY USE CASE AND IS NOT CORRECT IN GENERAL
                    l4 = self.criterion(T_hat, x_batch[:,time_delay:,:]) # T only
                else:
                    l4 = self.criterion(T_hat, x_batch[:,time_delay:,0:1]) # T only
            # L5: Worst case loss
            if c5 != 0:
                # l5 += torch.norm(x_batch[:,0,:]-x_hat[:,0,:], dim=1).max()
                # l5 += torch.norm(x_batch[:,1,:]-x_hat[:,1,:], dim=1).max()
                # l5 /= 2
                pass

            # Total loss
            train_loss = c1*l1 + c2*l2 + c4*l4 + c5*l5
            
            print(l2)
            print(l4)

            # Backward
            train_loss.backward()
            
            # Gradient clipping
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1)

            self.optimizer.step()

            if i == 0:
                smooth_train_loss = train_loss.item()
            else:
                smooth_train_loss = 0.99*smooth_train_loss + 0.01*train_loss.item()

            ####################################################################
            ### VAlIDATION
            ####################################################################
            if (i % train_args["val_every"] == 0):
                model.eval()

                batch_step = train_args["batch_size"]*(train_args["val_pred_steps"]+1)
                n_val_iter = int(x_val.size(0) / batch_step)

                val_loss = 0
                for j in range(n_val_iter):
                    val_ind = self._get_next_consec_index(
                        j, 
                        train_args["batch_size"], 
                        train_args["val_pred_steps"]+time_delay, 
                        x_val.size(0))

                    x_val_batch = x_val[val_ind, :] #x: [batch_size, pred_steps+time_delay+1, n_x] 
                    if self.is_autonomous == False:
                        u_val_batch = u_val[val_ind, :] #u: [batch_size, pred_steps+time_delay+1, n_u]  
                        d_val_batch = d_val[val_ind, :] #d: [batch_size, pred_steps+time_delay+1, n_d]

                        u_val_batch = u_val_batch[:,:train_args["val_pred_steps"]+time_delay,:] #u: [batch_size, pred_steps+time_delay, n_u]

                        # x:  0 1 2 ... k-1 k       -> length = k+1
                        # d:  0 1 2 ... k-1 k       -> length = k+1
                        # u:  0 1 2 ... k-1         -> length = k

                    x0      = x_val_batch[:,0:time_delay+1,:]
                    if self.is_autonomous == False:
                        d0      = d_val_batch[:,0:time_delay+1,:]
                        d_val_batch = d_val_batch[:,time_delay+1:,:]
                        u_val_batch = u_val_batch[:,time_delay:,:]

                        T_hat, _ = model.forward(x0, d0=d0, d=d_val_batch, u=u_val_batch)
                    else:
                        T_hat, _ = model.forward(x0, pred_steps = train_args["val_pred_steps"])

                    if self.is_autonomous: #THIS ONLY WORKS IN MY USE CASE AND IS NOT CORRECT IN GENERAL
                        val_loss += torch.nn.L1Loss()(T_hat[:,:,:], x_val_batch[:,time_delay:,:])
                    else:
                        val_loss += torch.nn.L1Loss()(T_hat[:,:,0], x_val_batch[:,time_delay:,0]) # T only

                if n_val_iter != 0:
                    val_loss /= n_val_iter

                ################################################################
                ## Print stats
                ################################################################

                if train_args["verbose"]:
                    pbar.set_description("lr={}, l_tot={:.4f}, l_val={:.4f}".format(
                        self.optimizer.param_groups[0]['lr'],
                        smooth_train_loss,
                        val_loss), refresh=True)

            ################################################################
            ### EARLY STOPPING
            ################################################################

            if train_args["early_stopping"]:
                if i == 0:
                    best_loss = val_loss
                if (val_loss < best_loss) or (i==0):
                    best_loss = val_loss
                    # Save parameters
                    self.best_model = model.state_dict()

            ####################################################################
            ## LEANRING RATE
            ####################################################################

            if i >= train_args["iter_enc_dec"]:
                self.lr_scheduler.step(smooth_train_loss)

            if train_args["min_lr"] >= self.optimizer.param_groups[0]['lr']:
                # Exit training loop
                print("Finish training as min_lr is attained.")
                if train_args["early_stopping"]:
                    model.load_state_dict(self.best_model)
                return model
        
        print("Finish training as max_iter is attained.")
        model.load_state_dict(self.best_model)
        return model

    def test(self, model, test_data, test_args):
        with torch.no_grad():
            model.eval()

            seed = test_args["seed"]
            torch.manual_seed(seed)

            state_scaler = MinMaxScaler()
            if self.is_autonomous == False:
                dist_scaler = MinMaxScaler()
                input_scaler = MinMaxScaler()

            state_scaler.__dict__ = model.state_scaler
            if self.is_autonomous == False:
                dist_scaler.__dict__ = model.dist_scaler
                input_scaler.__dict__ = model.input_scaler

            # Get test data
            x_test = state_scaler.transform(test_data['x'])
            if self.is_autonomous == False:
                d_test = dist_scaler.transform(test_data['d'])
                u_test = input_scaler.transform(test_data['u'])

            x_test = torch.Tensor(x_test)
            if self.is_autonomous == False:
                d_test = torch.Tensor(d_test)
                u_test = torch.Tensor(u_test)

            n_x = x_test.size(1)
            time_delay = test_args["encoding_time_delay"]

            # Get test batch
            test_index = self._create_rand_index(
                test_args["n_ics"],
                test_args["test_pred_steps"]+time_delay,
                x_test.size(0)
            )

            x_test_batch = x_test[test_index, :] #x: [batch_size, pred_steps+time_delay+1, n_x]
            if self.is_autonomous == False:
                d_test_batch = d_test[test_index, :] #u: [batch_size, pred_steps+time_delay+1, n_u]  
                u_test_batch = u_test[test_index, :] #d: [batch_size, pred_steps+time_delay+1, n_d]

                u_test_batch = u_test_batch[:,:test_args["test_pred_steps"]+time_delay,:] #u: [batch_size, pred_steps+time_delay, n_u]

                # x:  0 1 2 ... k-1 k       -> length = k+1
                # d:  0 1 2 ... k-1 k       -> length = k+1
                # u:  0 1 2 ... k-1         -> length = k

            # Forward
            x0      = x_test_batch[:,0:time_delay+1,:]
            if self.is_autonomous == False:
                d0 = d_test_batch[:,0:time_delay+1,:]
                d_test_batch = d_test_batch[:,time_delay+1:,:]
                u_test_batch = u_test_batch[:,time_delay:,:]

                T_hat, _ = model.forward(x0, d0=d0, d=d_test_batch, u=u_test_batch)
            else:
                T_hat, _ = model.forward(x0, pred_steps=test_args["test_pred_steps"])

            tmp = torch.zeros((T_hat.size(0)*T_hat.size(1),n_x))
            tmp[:,0] = T_hat.reshape(-1)
            T_hat = state_scaler.inverse_transform(tmp.detach().numpy())[:,0].reshape(test_args["n_ics"], test_args["test_pred_steps"]+1, 1)
            x_test_batch = state_scaler.inverse_transform(x_test_batch[:,time_delay:,:].reshape(-1, n_x)).reshape(test_args["n_ics"], test_args["test_pred_steps"]+1, n_x)

            T_hat = torch.Tensor(T_hat)
            x_test_batch = torch.Tensor(x_test_batch)

            mae = torch.nn.L1Loss(reduction="none")(T_hat, x_test_batch[:,:,0:1])
            mae = mae[:, :, 0] # Only select temperature error
            mae_over_pred_steps = torch.mean(mae, dim=0) # [test_pred_steps+1]
            average_mae_error = torch.mean(mae_over_pred_steps) # [1]
            if test_args["verbose"]:
                print("Average MAE: {}".format(average_mae_error))
        return average_mae_error, mae_over_pred_steps

    def save(self, model, path):
        params = {}
        params["state_dict"] = model.state_dict()
        params["state_scaler"] = model.state_scaler
        params["dist_scaler"] = model.dist_scaler
        params["input_scaler"] = model.input_scaler
        torch.save(params, path)

    def load(self, model, path):
        checkpoint = torch.load(path)
        model.load_state_dict(checkpoint["state_dict"])
        model.state_scaler = checkpoint["state_scaler"]
        model.dist_scaler = checkpoint["dist_scaler"]
        model.input_scaler = checkpoint["input_scaler"]
        
    @staticmethod
    @torch.jit.script
    def _create_rand_index(batch_size: int, pred_steps: int, data_len: int):
        """ Creates random indicies for one batch. Selects n=batch_size random
        initial conditions and the corresponding following samples (pred_steps).
        """
        index = torch.randint(0, data_len-(pred_steps+1), (batch_size,1))
        index = index.repeat(1,pred_steps+1) + torch.arange(0,pred_steps+1,1)
        return index.to(dtype=torch.long)

    @staticmethod
    @torch.jit.script
    def _get_next_consec_index(i: int, batch_size: int, pred_steps: int, data_len: int):
        batch_step = batch_size*(pred_steps+1)
        n_val_iter = int(data_len / batch_step)

        if i < n_val_iter:
            ind = torch.arange(i*batch_step,(i+1)*batch_step, pred_steps+1).reshape(-1,1)
            return ind.repeat(1,pred_steps+1) + torch.arange(0, pred_steps+1, 1)
        elif i == n_val_iter:
            last_batch_size = int((data_len - batch_step*n_val_iter) / (pred_steps+1))
            ind = torch.arange(batch_step*n_val_iter, batch_step*n_val_iter+last_batch_size*(pred_steps+1), pred_steps+1).reshape(-1,1)
            return ind.repeat(1,pred_steps+1) + torch.arange(0, pred_steps+1, 1)
        else:
            raise ValueError()