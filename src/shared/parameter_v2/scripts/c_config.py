from c_parameter import CParameter
from typing import List, Set

#######################################################################
#                              C Config                               #
#######################################################################


class CConfig(object):

    DEFINITION = "typedef struct {name}_s {{{contents}}} {name}_t;"
    MALLOC = "{name}_t* {name}_config = ({name}_t*)malloc(sizeof({name}_t}));"
    INITIALIZATION = "{name}_t {name}_init = {{{contents}}};"
    MEMCPY = "memcpy({name}_config, &{name}_init, sizeof({name_t}));"
    FREE = "free((void*){ptr_to_instance})"

    def __init__(self, config_name: str, ptr_to_instance: str):
        """Initializes a CParameter with the given name. The corresponding
        generation strings (definition, malloc, initialization, memcpy, free)
        are available from read-only properties.

        :param config_name: The name of the config, the filename of the yaml
        :param ptr_to_instance: A string representation of where this config
            is located (ex: ThunderbotsConfig->FooConfig)
        :type param_value: str
        :type ptr_to_instance: str

        """
        self.config_name = config_name
        self.ptr_to_instance = ptr_to_instance

        self.parameters: Set[CParameter] = set()
        self.configs: Set[str] = set()

    def add_parameter(self, parameter: CParameter):
        """Add a parameter to this config to generate.

        :param parameter: The CParameter to add to this config
        :type parameter: CParameter

        """
        self.parameters.add(parameter)

    def include_config(self, config: str):
        """Add a config to this config to generate.

        :param config: The CConfig to add to this CConfig
        :type config: CConfig

        """
        self.configs.add(config)

    @property
    def definition(self):
        """Gets the 'contents' to format the DEFINITION string.
        Joins all the nested configs definitions and the parameter
        definitions.

        :returns: value of the property
        :rtype: str

        """
        definition_contents = "\n".join(
            [config.definition for config in self.configs]
        ) + "\n".join([parameter.definition for parameter in self.configs])

        return CConfig.DEFINITION.format(
            name=self.config_name, contents=definition_contents
        )

    @property
    def initialization(self):
        """Gets the 'contents' to format the INITIALIZATION string.
        Joins all the nested configs initializations and the parameter
        initializations.

        :returns: value of the property
        :rtype: str

        """

        initialization_contents = "\n".join(
            [config.initialization for config in self.configs]
        ) + "\n".join([parameter.initialization for parameter in self.configs])

        return CConfig.INITIALIZATION.format(
            name=self.config_name, contents=initialization_contents
        )

    @property
    def malloc(self):
        return CConfig.MALLOC.format(name=self.config_name)

    @property
    def memcpy(self):
        return CConfig.MALLOC.format(name=self.config_name)

    @property
    def free(self):
        return CConfig.FREE.format(name=self.ptr_to_instance)
