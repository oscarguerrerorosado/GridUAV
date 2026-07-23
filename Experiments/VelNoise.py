import ast
import configparser
from pathlib import Path

from controller import Robot
from controller import Supervisor

from Utils import *

GridUAV_path = str(Path(__file__).parents[1])
print('Repository main folder -> {}'.format(GridUAV_path))

parameters = configparser.ConfigParser()
parameters.read(GridUAV_path + '/Experiments/config.ini')



class VelNoise_Experiment():
	def __init__(self):
		#super().__init__('velnoise_experiment')
		print('-------------- VelNoise Experiment --------------')
		print()

		self.exp_label = 'VelNoise'
		self.current_exp = 1

		self.trials_per_setting = int(parameters.get('Experiment', 'trials_per_setting'))
		self.sim_time = int(parameters.get('Experiment', 'sim_time'))
		self.noise_vel = ast.literal_eval(parameters.get('Experiment', 'noise_vel'))
		self.noise_act = ast.literal_eval(parameters.get('Experiment', 'noise_act'))

		self.starting_gain = float(parameters.get('GridConfig', 'starting_gain'))
		self.n_gains = ast.literal_eval(parameters.get('GridConfig', 'n_gains'))
		self.gain_spacing = ast.literal_eval(parameters.get('GridConfig', 'gain_spacing'))

		print("trials_per_setting parameter -> {}".format(self.trials_per_setting))
		print("sim_time parameter -> {}".format(self.sim_time))
		print("noise_vel parameter -> {}".format(self.noise_vel))
		print("noise_act parameter -> {}".format(self.noise_act))
		print()
		print("starting_gain parameter -> {}".format(self.starting_gain))
		print("n_gains parameter -> {}".format(self.n_gains))
		print("gain_spacing parameter -> {}".format(self.gain_spacing))



	def main_experiment_loop(self):
		# Do stuffs

		# Finish and save
		save_experiment_data(GridUAV_path, self.exp_label)



def main(args=None):
	robot = Robot()
	supervisor = Supervisor()
	robot_node = supervisor.getFromDef("Crazyflie")
	trans_field = robot_node.getField("translation")

	experiment = VelNoise_Experiment()
	experiment.main_experiment_loop()


if __name__ == '__main__':
	main()