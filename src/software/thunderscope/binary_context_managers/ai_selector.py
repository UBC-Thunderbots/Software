import os
from pathlib import Path
import tomllib
import logging
from software.thunderscope.constants import AISelectorConstants

#TODO integration with (#3568); seems really straightforward hopefully

#TODO integration with (3560); also seems straightforward I think. I think it might be easier if I wait till these two
# are merged to begin integrating? (though I did make comments in thunderscope main and setup sh)

class AIConfig:

    def __init__(
        self,
        chosen_blue: str,
        chosen_yellow: str,
    ) -> None:
        self.chosen_blue = chosen_blue
        self.chosen_yellow = chosen_yellow

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
        logging.info("No external AI")

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
        # Remove leading and trailing white space and return
        return file_path.strip()


def write_selected_ai(
        fullsystem_pair: AIConfig) -> None:

    """Creates a new TOML file named ai_config (if it doesn't already exist), and persists the absolute path
    of the selected blue and yellow binaries to it.

    @param chosen_blue the name of the selected AI binary for blue
    @param chosen_yellow: the name of the selected AI binary for yellow
    """

    blue_path = return_ai_path(fullsystem_pair.chosen_blue)
    yellow_path = return_ai_path(fullsystem_pair.chosen_yellow)

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

def read_saved_ai() -> AIConfig:

    """Returns a Fullsystem pair mapping to the AI binaries saved in the TOML file ai_config

    :return: a FullsystemPair object containing the blue and yellow AI binary paths
    """

    # Create default fullsystem pair with our fullsystem binaries
    fullsystem_pair = AIConfig(AISelectorConstants.FULLSYSTEM_PATH, AISelectorConstants.FULLSYSTEM_PATH)

    if os.path.isfile(AISelectorConstants.CONFIG_FILE_PATH):
        with open(AISelectorConstants.CONFIG_FILE_PATH, "rb") as file:
            selected_ai_dict = tomllib.load(file)
            # If a different blue fullsystem is persisted, replace the default arrangement
            if AISelectorConstants.BLUE_TOML_FIELD in selected_ai_dict.keys():
                fullsystem_pair.chosen_blue = selected_ai_dict.get(AISelectorConstants.BLUE_TOML_FIELD)
            # If a different yellow fullsystem is persisted, replace the default arrangemnet
            if AISelectorConstants.YELLOW_TOML_FIELD in selected_ai_dict.keys():
                fullsystem_pair.chosen_blue = selected_ai_dict.get(AISelectorConstants.YELLOW_TOML_FIELD)
            file.close()

    # Display logging message when using default fullsystem
    if fullsystem_pair.chosen_blue == AISelectorConstants.BLUE_TOML_FIELD:
        logging.info("TBots Fullsystem selected for blue side.")
    if fullsystem_pair.chosen_yellow == AISelectorConstants.YELLOW_TOML_FIELD:
        logging.info("TBots Fullsystem selected for yellow side.")

    return fullsystem_pair





