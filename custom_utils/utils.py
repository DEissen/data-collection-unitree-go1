import json
from os import path

def save_struct_as_json(path_measurement_dir, filename, struct_to_save):
    with open(path.join(path_measurement_dir, filename)) as fp:
        json.dump(struct_to_save, fp, indent=3)