import json
from os import path

def save_struct_as_json(path_measurement_dir, filename, dict_to_save):
    """
        Util function to save the dict dict_to_save as filename at path_measurement_dir.

        Parameters:
            - path_measurement_dir (str): Path to measurement dir where file shall be saved
            - filename (str): String with the filename for the JSON file
            - dict_to_save (dict): Dict which shall be saved in JSON format
    """
    with open(path.join(path_measurement_dir, filename), "w") as fp:
        json.dump(dict_to_save, fp, indent=3)

def get_floor_type_from_user():
    """
        Util function to let the user select a floor type by typing number in the console.

        Returns:
            - floor_type (str): String with the selected floor type
    """
    floor_types = ["invalid", "parquet", "road", "vinyl flooring", "tiles", "cobblestone", "grass"]
    not_determined_yet = True

    while not_determined_yet:
        user_input = input(f"Which floor type is this measurement done for?\n1 = {floor_types[1]}\n2 = {floor_types[2]}\n3 = {floor_types[3]}\n4 = {floor_types[4]}\nn5 = {floor_types[5]}\nn6 = {floor_types[6]}\n")

        # if conversion to int is not working it's an invalid input (can be determined by setting it to 0)
        try:
            user_input = int(user_input)
        except:
            user_input = 0

        if user_input == 1 or user_input == 2 or user_input == 3 or user_input == 4 or user_input == 5 or user_input == 6:
            not_determined_yet = False
        else:
            print("\nInvalid input, please try again!\n")
    
    return floor_types[user_input]


# main for testing only
if __name__ == "__main__":
    pass