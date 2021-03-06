from notebook.notebookapp import main
from notebook.nbextensions import install_nbextension_python, enable_nbextension_python
from notebook.serverextensions import toggle_serverextension_python
import sys
import subprocess

if __name__ == "__main__":
    # a very nasty hack to install the jupytext server extension into the hermetic interpreter
    # we CANNOT do this with bazel because adding jupytext to the requirements.txt and then
    # adding it as a dependency causes a bazel dependency cycle
    # we do NOT install this package with '--user' because it alters files outside of the
    # bazel hermetic interpreter
    subprocess.check_call([sys.executable, "-m", "pip", "install", "jupytext==1.10.1"])
    # Sometimes jupyter cannot figure out that jupytext is installed, and the .py notebooks
    # won't load, requiring a restart of this script. This possibly remedies this issue.
    import jupytext

    # enable the jupytext server extension
    toggle_serverextension_python("jupytext")

    # install and enable the jupytext notebook extension
    install_nbextension_python("jupytext", user=True)
    enable_nbextension_python("jupytext")

    # we have to manually enable the "widgetsnbextension" notebook extension to use ipywidgets inside our hermetic
    # python interpreter
    install_nbextension_python("widgetsnbextension", user=True)
    enable_nbextension_python("widgetsnbextension")
    sys.exit(main())
