from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar

from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from proto.import_all_protos import *
import qtawesome as qta

class GLGenerateTestToolbar:

    def __init__(
        self, toolbar: GLToolbar,
        buffer_size: int = 5
    ):
        self.__toolbar = toolbar

        self.world_buffer = ThreadSafeBuffer(buffer_size, World)

        self.test_generate_button = self.__toolbar.setup_icon_button(
            qta.icon("mdi6.download"),
            "Converts the current robot configurations to a PyTest test case",
            self.__generate_test,
            display_text="Convert to PyTest",
        )
        toolbar.add_button(self.test_generate_button)

        self.cached_world = None

    def refresh(self):
        self.cached_world = self.world_buffer.get(block=False, return_cached=True)

    def __generate_test(self):
        pass


