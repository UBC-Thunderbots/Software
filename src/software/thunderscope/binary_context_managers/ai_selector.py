import os
from pathlib import Path
import tomllib
from software.thunderscope.constants import AISelectorConstants

#TODO integration with (#3568); seems really straightforward hopefully

#TODO integration with (3560); also seems straightforward I think. I think it might be easier if I wait till these two
# are merged to begin integrating? (though I did make comments in thunderscope main and setup sh)


def find_available_ai() -> list[str]:
    """Returns a list of all available binary names, including our fullsystem

    :return: A list of all available binary names, or just a list with our fullsystem if an error occurs
    """
    list_of_ai = []
    try:
        # Check for all executable files in the folder
        for file_name in os.listdir(AISelectorConstants.AI_FOLDER):
            file_path = os.path.join(AISelectorConstants.AI_FOLDER, file_name)
            if os.path.isfile(file_path) and os.access(file_path, os.X_OK):
                list_of_ai.append(os.path.basename(file_name))

    except (FileNotFoundError, PermissionError, NotADirectoryError):
        pass #TODO not sure what behaviour to put here, maybe a debug message or something (if it's even necessary)?

    finally:
        # Add an option for our fullsystem
        list_of_ai.append(AISelectorConstants.FULLSYSTEM_DISPLAY_NAME)
        return list_of_ai

def return_ai_path(selected_ai: str) -> str:
    """Returns the absolute path of a binary given its name, or the path of our default fullsystem if the binary is not
    valid.
    :param selected_ai: the name of the selected AI binary

    :return: the absolute path of the binary as a string
    """

    file_path = os.path.join(AISelectorConstants.AI_FOLDER, selected_ai)
    # Default to our full system if it is selected or the selected binary doesn't exist
    if selected_ai == AISelectorConstants.FULLSYSTEM_DISPLAY_NAME or not os.path.isfile(file_path):
        return AISelectorConstants.FULLSYSTEM_PATH
    else:
        # Remove all white space and return
        #TODO not sure if that is necessary, but probably doesn't hurt?
        return "".join(file_path.split())


def write_selected_ai(
        chosen_blue: str,
        chosen_yellow: str) -> None:

    """Creates a new TOML file named ai_config (if it doesn't already exist), and persists the absolute path
    of the selected blue and yellow binaries to it.

    @param chosen_blue the name of the selected AI binary for blue
    @param chosen_yellow: the name of the selected AI binary for yellow
    """

    blue_path = return_ai_path(chosen_blue)
    yellow_path = return_ai_path(chosen_yellow)

    """Format in TOML as:
    blue_path_to_binary: '<absolute path>'
    yellow_path_to_binary: '<absolute path>'"""

    selected_ais = (
        f'{AISelectorConstants.BLUE_TOML_FIELD} = "{blue_path}"\n'
        f'{AISelectorConstants.YELLOW_TOML_FIELD} = "{yellow_path}"\n'
    )

    # create a new config file if it doesn't exist, and write in the format above to it

    with open(AISelectorConstants.CONFIG_FILE_PATH, "w") as file:
        Path(AISelectorConstants.CONFIG_FILE_PATH).write_text(selected_ais, encoding="utf-8")
        file.close()

def read_saved_ai() -> dict[str, str]:

    """Returns a string dictionary mapping to the AI binaries saved in the TOML file ai_config

    :return: a string dictionary of the blue and yellow AI binary paths
    """

    saved_blue = None
    saved_yellow = None

    if not os.path.isfile(AISelectorConstants.CONFIG_FILE_PATH):
        # If there is no config file, default to our fullsystem
        saved_blue = AISelectorConstants.FULLSYSTEM_PATH
        saved_yellow = AISelectorConstants.FULLSYSTEM_PATH
    else:
        with open(AISelectorConstants.CONFIG_FILE_PATH, "rb") as file:
            # If the TOML File formatting is invalid, default to our fullsystem
            selected_ai_dict = tomllib.load(file)
            if AISelectorConstants.BLUE_TOML_FIELD in selected_ai_dict.keys():
                saved_blue = selected_ai_dict.get(AISelectorConstants.BLUE_TOML_FIELD)
            else:
                saved_blue = AISelectorConstants.FULLSYSTEM_PATH
            if AISelectorConstants.YELLOW_TOML_FIELD in selected_ai_dict.keys():
                saved_yellow = selected_ai_dict.get(AISelectorConstants.YELLOW_TOML_FIELD)
            else:
                saved_yellow = AISelectorConstants.FULLSYSTEM_PATH
            file.close()

    # Return a dictionary of colour to the path string
    saved_ais = {
        "Blue": saved_blue,
        "Yellow": saved_yellow
    }

    #TODO not sure if this should be a dictionary or something like a tuple
    return saved_ais





