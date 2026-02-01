from software.thunderscope.gl.widgets.gl_toolbar import GLToolbar
from software.thunderscope.gl.widgets.toolbars.runtime.gl_runtime_installer import (
    GLRuntimeInstallerDialog,
)
from software.thunderscope.gl.widgets.toolbars.runtime.gl_runtime_selector import (
    GLRuntimeSelectorDialog,
)
import qtawesome as qta


class GLRuntimeToolbar:
    """A toolbar with controls for Installing Runtimes"""

    def __init__(self, toolbar: GLToolbar):
        self.__toolbar = toolbar

        self.runtime_installer_button = self.__toolbar.setup_icon_button(
            qta.icon("mdi6.download"),
            "Opens a runtime installer modal",
            self.__open_runtime_installer_dialog,
            display_text="Install Runtimes",
        )

        self.runtime_selector_button = self.__toolbar.setup_icon_button(
            qta.icon("mdi6.server"),
            "Select runtimes for each team",
            self.__open_runtime_selector_dialog,
            display_text="Select Runtimes",
        )

        toolbar.add_button(self.runtime_installer_button)
        toolbar.add_button(self.runtime_selector_button)

    def __open_runtime_installer_dialog(self) -> None:
        """Opens the runtime installer modal, initializing if first time"""
        if not hasattr(self, "runtime_installer_dialog"):
            self.runtime_installer_dialog = GLRuntimeInstallerDialog(
                parent=self.parent()
            )

        self.runtime_installer_dialog.show()

    def __open_runtime_selector_dialog(self) -> None:
        """Initializes and opens the runtime selector dialog"""
        self.runtime_selector_dialog = GLRuntimeSelectorDialog(parent=self.parent())
        self.runtime_selector_dialog.show()
