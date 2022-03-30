import do_mpc
from casadi import *
from scipy.constants import convert_temperature
import datetime
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter, ImageMagickWriter

VALIDATION_START_DATE = datetime.datetime(2021, 3, 7, 0, 0)

def mpc2(mpc_model, mpc_params, system, datatransformer):
    A = mpc_model["ss_model"]["A"]
    B_d = mpc_model["ss_model"]["B_d"]
    B_u = mpc_model["ss_model"]["B_u"]
    B_uz = mpc_model["ss_model"].get("B_uz", None)
    C = mpc_model["ss_model"]["C"]
    D_d = mpc_model["ss_model"]["D_d"]
    D_u = mpc_model["ss_model"]["D_u"]

    time_delay = mpc_model["time_delay"]

    # Convert everything into casadi objects
    A, B_u, B_d = DM(A), DM(B_u), DM(B_d)
    C, D_u, D_d = DM(C), DM(D_u), DM(D_d)
    if B_uz is not None:
        B_uz = DM(B_uz)

    # Get encoding function
    encode = mpc_model["encode_fn"]

    # Get scalers
    state_scaler = mpc_model["state_scaler"]
    dist_scaler = mpc_model["dist_scaler"]
    input_scaler = mpc_model["input_scaler"]
    
    # Dimensions
    transformer = datatransformer()
    n_x, n_d, n_u = transformer.get_dims()
    n_z, n_y = A.shape[0], C.shape[0]

    ############################################################################
    ### SYSTEM
    ############################################################################

    # Initialization
    system = system(sim_days = mpc_params["sim_days"]+1, start_date=VALIDATION_START_DATE)

    # Initial states
    out_prev = system.get_current_state()
    out = system.make_step(0)
    forecast = system.get_weather_forecast(24)

    # data from t=0
    x_unscaled, d_unscaled, _ = transformer.envOut2vars(out)
    x_scaled = state_scaler.transform(x_unscaled)
    d_scaled = dist_scaler.transform(d_unscaled)
    
    if time_delay:
        # data from t=-1
        x0_unscaled, d0_unscaled, _ = transformer.envOut2vars(out_prev)
        x_prev_scaled = state_scaler.transform(x0_unscaled)
        d_prev_scaled = dist_scaler.transform(d0_unscaled)

        x_scaled = np.concatenate((x_prev_scaled, x_scaled))
        d_scaled = np.concatenate((d_prev_scaled, d_scaled))

    z0 = encode(x_scaled, d_scaled)

    forecast = transformer.forcast2vars(forecast)
    forecast = dist_scaler.transform(forecast)

    ############################################################################
    ### MPC MODEL
    ############################################################################

    model_type  = 'discrete'
    mpc_model   = do_mpc.model.Model(model_type)

    N_hours     = 6             #[h]
    N_steps     = N_hours * 4   #MPC horizon [steps]
    time_step   = 0.25          #time step [h]
    delta_T     = 1             #comfort temperature intervall [°C]
    delta_T_night = 1           #night setback intervall [°C]
    T_set       = 21            #set temperature [°C]
    T_set = convert_temperature(T_set, 'Celsius', 'Kelvin') #set temperature [K]
    
    print("\n### SETTINGS ### ")
    print(f"Set temperature: {T_set}")

    T_lb = T_set - delta_T # temperature lower bound wo night setback
    T_ub = T_set + delta_T # temperature upper bound wo night setback
    print(f"Lower temperature bound (day): {T_lb}\nUpper temperature bound (day): {T_ub}")
    tmp = np.zeros((1, n_x))
    tmp[0] = T_lb
    T_lb = state_scaler.transform(tmp)[0,0]
    tmp[0] = T_ub
    T_ub = state_scaler.transform(tmp)[0,0]
    
    tmp[0] = T_set + delta_T + delta_T_night
    delta_T_night_scal = state_scaler.transform(tmp)[0,0] - T_ub

    ### Define states
    _z = mpc_model.set_variable(var_type='_x', var_name='z', shape=(n_z,1))
    _u_prev = mpc_model.set_variable(var_type='_x', var_name='u_prev', shape=(1,1))

    ### Define inputs
    _u = mpc_model.set_variable(var_type='_u', var_name='u', shape=(n_u,1))

    ### Define outputs
    _y = mpc_model.set_expression('y', C@_z)

    ### Define time varying parameters (tvp), e.g. external disturbances
    _d = mpc_model.set_variable(var_type='_tvp', var_name='d', shape=(n_d,1)) #weather
    _n = mpc_model.set_variable(var_type='_tvp', var_name='n', shape=(1,1)) #is-night variable

    ### Define System dynamics
    if B_uz is None:
        z_next = A@_z + B_d@_d + B_u@_u
    else:
        z_next = A@_z + B_d@_d + B_u@_u + B_uz@_z@_u
    mpc_model.set_rhs('z', z_next)
    mpc_model.set_rhs('u_prev', _u)

    # Set auxillary expressions
    _T_lb = mpc_model.set_expression('T_lb', T_lb-delta_T_night_scal*_n)
    _T_ub = mpc_model.set_expression('T_ub', T_ub+delta_T_night_scal*_n)

    # Finish model setup
    mpc_model.setup()

    ############################################################################
    ### MPC PROBLEM
    ############################################################################

    mpc = do_mpc.controller.MPC(mpc_model)

    # Setup controller
    setup_mpc = {
        'n_horizon': N_steps,
        't_step': time_step,
        'store_full_solution': True,
        'nl_cons_single_slack': False,
        'store_lagr_multiplier': False,
        'nlpsol_opts': {'ipopt.linear_solver': 'mumps',
                        'ipopt.print_level': 0, 
                        'print_time': 0}, #linear solver
    }
    mpc.set_param(**setup_mpc)

    h, m = 0, 0
    # Define evolution of time_varying parameters
    tvp_template = mpc.get_tvp_template()
    def tvp_fun(t_now):
        t = datetime.datetime.combine(datetime.date.today(), datetime.time(h, m))
        for i in range(N_steps):
            is_night = 1 if t.hour > 19 or t.hour < 6 else 0
            tvp_template['_tvp',i,'n'] = is_night
            tvp_template['_tvp',i,'d'] = forecast[i]
            t += datetime.timedelta(minutes=60*time_step)
        return tvp_template
    mpc.set_tvp_fun(tvp_fun)

    # Define constraints
    mpc.bounds['lower','_u','u'] = 0
    mpc.bounds['upper','_u','u'] = 1

    mpc.set_nl_cons('temperature_lb', -C@_z+T_lb-delta_T_night_scal*_n, ub=0, soft_constraint=False)
    mpc.set_nl_cons('temperature_ub', C@_z-(T_ub+delta_T_night_scal*_n), ub=0, soft_constraint=False)

    du = 0.2
    mpc.set_nl_cons('delta_u_ub', _u - _u_prev, ub=du, soft_constraint=False)
    mpc.set_nl_cons('delta_u_lb', _u_prev - _u, ub=du, soft_constraint=False)

    # Objective function 
    lterm = _u**2 # stage cost
    mterm = DM(np.zeros((1,1))) # terminal cost (function of _x, _tvp, _p)
    mpc.set_objective(mterm=mterm, lterm=lterm)
    # mpc.set_rterm(u = 1e+3) # delta input cost

    # Finish MPC setup
    mpc.setup()

    ############################################################################
    ### SIMULATION 
    ############################################################################

    ### PREPARE GIF
    if mpc_params["create_gif"]:
        mpc_graphics = do_mpc.graphics.Graphics(mpc.data)

        fig = plt.figure(figsize=(16,9))

        ax0 = plt.subplot2grid((3, 1), (0, 0))
        ax1 = plt.subplot2grid((3, 1), (1, 0))
        ax2 = plt.subplot2grid((3, 1), (2, 0))

        for ax in [ax0, ax1, ax2]:
            ax.yaxis.set_label_position("right")
            ax.yaxis.tick_right()
            ax.grid()
            if ax != ax2:
                ax.xaxis.set_ticklabels([])

        ax2.set_xlabel('time steps')
        ax2.set_ylim(-0.1, 1.1)

        mpc_graphics.add_line(var_type='_aux', var_name='y', axis=ax0)
        mpc_graphics.add_line(var_type='_aux', var_name='T_lb', axis=ax0, color='r', linestyle='-')
        mpc_graphics.add_line(var_type='_aux', var_name='T_ub', axis=ax0, color='r', linestyle='-')

        mpc_graphics.add_line(var_type='_tvp', var_name='d', axis=ax1)
        mpc_graphics.add_line(var_type='_u', var_name='u', axis=ax2)

        fig.align_ylabels()
        fig.tight_layout()

    # Parameters
    sim_steps = int(mpc_params["sim_days"] * 24 * 4)

    mpc.x0 = np.vstack((z0, np.zeros((1,1))))
    mpc.set_initial_guess()
    mpc.reset_history()

    # Closed loop
    u_scaled = np.zeros((1,1))
    u_tot = 0
    for k in range(sim_steps):
        print(f"### Iter {k} ###")
        # Lifting initial state
        x, d, _ = transformer.envOut2vars(out)
        print(f"x {x}")
        print(f"d {d}")
        x_scaled = state_scaler.transform(x)
        d_scaled = dist_scaler.transform(d)
        print(f"x_scaled {x}")
        print(f"d_scaled {d}")
        if time_delay:
            z = encode(np.concatenate((x_prev_scaled, x_scaled)), np.concatenate((d_prev_scaled, d_scaled)))
        else:
            z = encode(x_scaled, d_scaled)
        z = np.vstack((z, u_scaled))
        print(f"z {z.T}")

        # Find out if it's at night
        m, h = system.get_time()

        # Get weather forecast
        forecast = system.get_weather_forecast(forecast_length=N_steps)
        forecast = transformer.forcast2vars(forecast)
        forecast = dist_scaler.transform(forecast)
        print(f"forecast {forecast.T}")

        # Get control signal and advance one step
        u_scaled = mpc.make_step(z)
        u_unscaled = input_scaler.inverse_transform(u_scaled)
        u = u_unscaled[0,0]

        u_tot += u

        # Apply input 
        out = system.make_step(u)

        x_prev_scaled = x_scaled
        d_prev_scaled = d_scaled

        ##### DEBUGGING START
        # print(mpc.data._x[:,0])
        # print(mpc.data._x.shape)
        # print(A)
        # print(z[0:n_z])
        # print(f"z_k+1 = A*z+B_d*d = {(A@z[0:n_z]+B_d@d.T)[0]}")
        # print(C)
        print(f"C@z {C@z[0:n_z]}")
        # print(z.sum())
        ##### DEBUGGING END

    T = mpc.data._x[:,0]
    temp = np.zeros((T.shape[0], n_x))
    temp[:,0] = T
    T = state_scaler.inverse_transform(temp)[:,0]
    print(T)

    ############################################################################
    ### EVALUATION 
    ############################################################################
     
    ub = mpc.data._aux[:,3]
    lb = mpc.data._aux[:,2]
    temp = np.zeros((ub.shape[0], n_x))
    temp[:,0] = ub
    ub = state_scaler.inverse_transform(temp)[:,0]
    temp[:,0] = lb
    lb = state_scaler.inverse_transform(temp)[:,0]

    T = mpc.data._aux[:,1] #changed for
    # T = mpc.data._x[:,0]
    temp = np.zeros((T.shape[0], n_x))
    temp[:,0] = T
    T = state_scaler.inverse_transform(temp)[:,0]

    constr_violation_lb = T < lb
    constr_violation_ub = T > ub

    constr_violation = np.any((constr_violation_lb, constr_violation_ub), axis=0)
    mag_constr_violation_lb = np.abs(T - lb) * constr_violation_lb
    mag_constr_violation_ub = np.abs(T - ub) * constr_violation_ub
    mag_constr_violation = mag_constr_violation_lb.sum() + mag_constr_violation_ub.sum()
    
    n_constr_violation = constr_violation.sum()

    stats = {}
    stats['av_energy_per_step'] = u_tot/sim_steps
    stats['nb_constr_violation'] = n_constr_violation
    stats['av_mag_of_constr_violation'] = mag_constr_violation/n_constr_violation

    print("\n### Evaluation ###")
    print("Av. energy / step [%]  : {}".format(u_tot/sim_steps))
    print("# constr. violation [] : {}".format(n_constr_violation))
    print("Av. mag. of. constr. vio. [K] : {}".format(mag_constr_violation/n_constr_violation if n_constr_violation !=0 else 0))

    ############################################################################
    ### PLOTTING 
    ############################################################################    

    ### GIF
    def update(t_ind):
        mpc_graphics.plot_results(t_ind)
        mpc_graphics.plot_predictions(t_ind)
        mpc_graphics.reset_axes()
   
    if mpc_params["create_gif"]:
        anim = FuncAnimation(fig, update, frames=sim_steps, repeat=False)
        gif_writer = ImageMagickWriter(fps=20)
        anim.save('test.gif', writer=gif_writer)

    ### STATIC PLOT
    if mpc_params["plot_results"]:
        # Plot in original domain
        fig, ax = plt.subplots(3, sharex=True, figsize=(16,9))
        ax[0].plot(ub, color='r')
        ax[0].plot(lb, color='r')
        ax[0].plot(T, linestyle="--")
        ax[0].set_ylabel("T [K]")
        ax[0].grid()

        ax[1].plot(mpc.data._tvp[:,0])
        ax[1].set_ylabel("T_out []")
        ax[1].grid()

        ax1_right = ax[1].twinx()
        ax1_right.plot(mpc.data._tvp[:,1], color="orange")
        ax1_right.set_ylabel("rad []")

        ax[2].plot(mpc.data._u)
        ax[2].set_ylabel("u []")
        ax[2].set_xlabel("time step (1 step = 15min)")
        ax[2].set_ylim(-0.1,1.1)
        ax[2].grid()
        plt.show()

    return mpc.data, stats