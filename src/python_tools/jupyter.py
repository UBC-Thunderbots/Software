from notebook.notebookapp import main
from notebook.nbextensions import install_nbextension_python, enable_nbextension_python
from notebook.serverextensions import toggle_serverextension_python
import sys
import subprocess

if __name__ == "__main__":
    # enable the jupytext server extension
    toggle_serverextension_python("jupytext", enabled=True)

    # install and enable the jupytext notebook extension
    install_nbextension_python("jupytext", user=True)
    enable_nbextension_python("jupytext")

    # we have to manually enable the "widgetsnbextension" notebook extension to use ipywidgets inside our hermetic
    # python interpreter
    install_nbextension_python("widgetsnbextension", user=True)
    enable_nbextension_python("widgetsnbextension")
    sys.exit(main(argv=["--no-browser"]))
