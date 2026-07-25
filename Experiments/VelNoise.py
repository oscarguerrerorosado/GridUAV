import ast
import configparser
import sys
from pathlib import Path

from Utils import *

GridUAV_path = str(Path(__file__).parents[1])
print('Repository main folder -> {}'.format(GridUAV_path))

if GridUAV_path not in sys.path:
	sys.path.insert(0, GridUAV_path)

from RobotControl.WebotsController import WebotsController

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

		self.movement_pattern = parameters.get(
			'Movement_pattern', 'Movement_pattern', fallback='random_3d'
		).strip() or 'random_3d'

		print("trials_per_setting parameter -> {}".format(self.trials_per_setting))
		print("sim_time parameter -> {}".format(self.sim_time))
		print("noise_vel parameter -> {}".format(self.noise_vel))
		print("noise_act parameter -> {}".format(self.noise_act))
		print()
		print("starting_gain parameter -> {}".format(self.starting_gain))
		print("n_gains parameter -> {}".format(self.n_gains))
		print("gain_spacing parameter -> {}".format(self.gain_spacing))
		print("Movement_pattern parameter -> {}".format(self.movement_pattern))



	def main_experiment_loop(self):
		# Simulation ownership (Robot + time loop) lives in WebotsController.
		webots = WebotsController(sim_time=self.sim_time, pattern=self.movement_pattern)
		webots.run()

		# Finish and save
		save_experiment_data(GridUAV_path, self.exp_label)



def main(args=None):
	experiment = VelNoise_Experiment()
	experiment.main_experiment_loop()


if __name__ == '__main__':
	main()
