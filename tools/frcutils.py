import json
import os


def read_file(directory, file_name):
    """Find file and return contents.

    Checks current directory for file. If one doesn't exist, try all parent
    directories as well.

    Keyword arguments:
    directory -- current directory from which to start search
    file_name -- file name string

    Returns string containing file contents or triggers program exit.
    """
    while True:
        try:
            with open(directory + os.sep + file_name, "r") as file_contents:
                return file_contents.read()
        except OSError:
            # .git files are ignored, which are created within submodules
            if os.path.isdir(directory + os.sep + ".git"):
                print(f"Error: file '{file_name}' not found in '{directory}'")
                sys.exit(1)
            directory = os.path.dirname(directory)


def get_roborio_ip():
    """Returns the team's roboRIO IP address."""
    prefs = read_file(os.getcwd(), ".wpilib/wpilib_preferences.json")
    team = str(json.loads(prefs)["teamNumber"])
    return f"10.{team[:2]}.{team[2:]}.2"
