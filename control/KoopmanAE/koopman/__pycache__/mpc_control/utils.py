import energym
from energym.wrappers.downsample_outputs import DownsampleOutputs
import numpy as np
import datetime

class SimpleHouseSystem:
    " Wrapper for energym model "
    def __init__(self, sim_days, start_date):
        " Constructor of SimpleHouseWrapper. "
        self.sim_days = sim_days
        self.dt = 15 #time step [min]
        self.sim_steps = int(sim_days * 24 * 60 / self.dt)
        self.downsampling_steps = int(self.dt / 5)

        weather = "CH_ZH_Maur"
        self.env = energym.make("SimpleHouseRad-v0", weather=weather, simulation_days=sim_days,
            start_day=start_date.day, start_month=start_date.month, year=start_date.year)

        if self.downsampling_steps > 1:
            downsampling_dic = {}
            for key in self.env.get_outputs_names():
                downsampling_dic[key] = np.mean
            self.env = DownsampleOutputs(self.env, steps=self.downsampling_steps, downsampling_dic=downsampling_dic)

        # Do one first step
        self.make_step(np.float64(0.5))

    def get_current_state(self):
        return self.env.get_output()

    def get_time(self):
        " Gets current time of model. "
        m, h,_,_ = self.env.get_date()
        return m, h

    def close_env(self):
        self.env.close()

    def make_step(self, u):
        " Go one time step ahead. "
        control = {}
        control['u'] = [u]
        return self.env.step(control)

    def get_weather_forecast(self, forecast_length):
        """ Get weather forecast 
        Parameters:
            forecast_length. int
                Number of hours to get forecast for. 
        """
        forecast = self.env.get_forecast(forecast_length=forecast_length)
        return forecast

class SwissHouseSystem:
    " Wrapper for energym model "
    def __init__(self, sim_days, start_date):
        " Constructor of SwissHouseWrapper. "
        self.sim_days = sim_days
        self.dt = 15 #time step [min]
        self.sim_steps = int(sim_days * 24 * 60 / self.dt)
        self.downsampling_steps = int(self.dt / 5)

        weather = "CH_ZH_Maur"
        self.env = energym.make("SwissHouseRSlaW2W-v0", weather=weather, simulation_days=sim_days,
            start_day=start_date.day, start_month=start_date.month, year=start_date.year)

        if self.downsampling_steps > 1:
            downsampling_dic = {}
            for key in self.env.get_outputs_names():
                downsampling_dic[key] = np.mean
            self.env = DownsampleOutputs(self.env, steps=self.downsampling_steps, downsampling_dic=downsampling_dic)

        # Do one first step
        self.make_step(np.float64(0))

    def get_current_state(self):
        return self.env.get_output()

    def get_time(self):
        " Gets current time of model. "
        m, h,_,_ = self.env.get_date()
        return m, h

    def close_env(self):
        self.env.close()

    def make_step(self, u):
        " Go one time step ahead. "
        control = {}
        control['u'] = [u]

        return self.env.step(control)

    def get_weather_forecast(self, forecast_length):
        """ Get weather forecast 
        Parameters:
            forecast_length. int
                Number of hours to get forecast for. 
        """
        forecast = self.env.get_forecast(forecast_length=forecast_length)
        return forecast