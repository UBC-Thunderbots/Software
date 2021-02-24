from software.python_bindings import passing

passing_config = passing.getPassingConfig()
passing_config

passing_config["num_passes_to_optimize"] = 69
passing.updatePassingConfigFromDict(passing_config)
passing.getPassingConfig()
