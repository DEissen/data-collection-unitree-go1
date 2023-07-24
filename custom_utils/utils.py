import json
from os import path

def save_struct_as_json(path_measurement_dir, filename, struct_to_save):
    with open(path.join(path_measurement_dir, filename), "w") as fp:
        json.dump(struct_to_save, fp, indent=3)

def get_floor_type_from_user():
    floor_types = ["invalid", "grass", "gravel", "vinyl flooring", "tiles"]
    not_determined_yet = True

    while not_determined_yet:
        user_input = input(f"Which floor type is this measurement done for?\n1 = {floor_types[1]}\n2 = {floor_types[2]}\n3 = {floor_types[3]}\n4 = {floor_types[4]}\n")

        # if conversion to int is not working it's an invalid input (can be determined by setting it to 0)
        try:
            user_input = int(user_input)
        except:
            user_input = 0

        if user_input == 1 or user_input == 2 or user_input == 3 or user_input == 4:
            not_determined_yet = False
        else:
            print("\nInvalid input, please try again!\n")
    
    return floor_types[user_input]


# main for testing only
if __name__ == "__main__":
    pass