import os
import shutil

def save_experiment_data(ws_path, exp_label):
	data_path = os.path.join(ws_path, "Data")
	exp_number = 1
	exp_folder = "{}_Exp{}".format(exp_label, exp_number)

	folders = [name for name in os.listdir(data_path) if os.path.isdir(os.path.join(data_path, name))]
	
	print("Saving experimental data")

	while exp_folder in folders:
		exp_number += 1
		exp_folder = "{}_Exp{}".format(exp_label, exp_number)

	print('Creating new data folder =', exp_folder)
	exp_folder_path = os.path.join(data_path, exp_folder)
	os.makedirs(exp_folder_path)

	config_src = os.path.join(ws_path, "Experiments", "config.ini")
	config_dst = os.path.join(exp_folder_path, "config.ini")
	shutil.copy(config_src, config_dst)
	print("Copied config.ini to", config_dst)