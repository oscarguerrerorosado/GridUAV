import os
import configparser
from pathlib import Path

ws_path = str(Path(__file__).parents[1])
print('Repository main folder -> {}'.format(ws_path))

parameters = configparser.ConfigParser()
parameters.read(ws_path + '/Experiments/config.ini')

webots_world = int(parameters.get('Experiment', 'webots_world'))

print("Webots world parameter -> {}".format(webots_world))



def save_experiment_data(ws_path, param1, param2, param3, param4):
	data_path = ws_path + "/Data/"
	exp_folder = "/Exp1_param1{}_param2{}_param3{}_param4{}".format(param1, param2, param3, param4)
	folders = [name for name in os.listdir(data_path) if os.path.isdir(os.path.join(data_path, name))]
	print("exp_folder", exp_folder)
	print("folders", folders)

	if (exp_folder in folders) == False:
        print('Creating new data folder =', exp_folder)
        os.makedirs(data_path + '/' + exp_folder)

save_experiment_data(ws_path, 1.0, 2.0, 3.0, 4.0)