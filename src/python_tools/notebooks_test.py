import unittest
import os
import glob
import subprocess
import sys

# This test does a simple sanity check to make sure that all the notebooks run, and therefore, that all of the bindings
# that are used in them are functional.


class NotebooksTestCase(unittest.TestCase):
    def testAllNotebooksRun(self):
        notebooks_dir = os.path.join(os.getcwd(), "python_tools/notebooks")
        for notebook in glob.glob(os.path.join(notebooks_dir, "*.py"), recursive=False):
            notebook_path = os.path.join(notebooks_dir, notebook)
            print(f"Found notebook {notebook_path}")
            subprocess.check_call(
                [sys.executable, "-m", "IPython", notebook_path], cwd=notebooks_dir
            )


if __name__ == "__main__":
    unittest.main()
