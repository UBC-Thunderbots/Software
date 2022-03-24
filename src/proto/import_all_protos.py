import importlib
import pkgutil
import proto


def get_top_level_modules(package):
    """Grab all modules that are 1 level deep.

    :param package: package (name or actual module) to grab from
    :returns: list[str]

    """
    results = []

    for loader, name, is_pkg in pkgutil.walk_packages(package.__path__):
        full_name = package.__name__ + "." + name
        results.append(full_name)

    return results


def import_all_classes(package, input_globals):
    """Import all classes from the given package, specifically useful
    for autogenerated protobuf modules.

    :param package: The package to import from
    :param input_globals: globals() from where this is called

    """
    results = get_top_level_modules(package)

    for result in results:
        module = importlib.import_module(result)

        # is there an __all__?  if so respect it
        if "__all__" in module.__dict__:
            names = module.__dict__["__all__"]
        else:
            # otherwise we import all names that don't begin with _
            names = [x for x in module.__dict__ if not x.startswith("_")]

        # update the globals to contain the class
        input_globals.update({k: getattr(module, k) for k in names})


# Import all the protobuf classes that are generated into the proto library
import_all_classes(proto, globals())

# Now add the following line to get access to all protobufs
# from proto.import_all_protos import *
