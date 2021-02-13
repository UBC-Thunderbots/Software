from notebook.notebookapp import main
from notebook.nbextensions import install_nbextension_python, enable_nbextension_python
import sys

if __name__ == "__main__":
    # we have to manually enable the "widgetsnbextension" notebook extension to use ipywidgets inside our hermetic
    # python interpreter
    install_nbextension_python("widgetsnbextension", user=True)
    enable_nbextension_python("widgetsnbextension")
    sys.exit(main())
