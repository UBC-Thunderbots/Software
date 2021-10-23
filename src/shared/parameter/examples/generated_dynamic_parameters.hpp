// This is an example for what a generated dynamic parameters file would look like, based
// on the configs found in /src/shared/parameter/config_definitions/test

#pragma once
#include <boost/program_options.hpp>
#include <iostream>
#include <optional>

#include "software/ai/hl/stp/play/play.h"
#include "software/parameter/config.h"
#include "software/parameter/enumerated_parameter.h"
#include "software/parameter/numeric_parameter.h"
#include "software/util/generic_factory/generic_factory.h"

class fooConfig;
class exampleConfig;

class fooConfig : public Config
{
   public:
    fooConfig()
    {
        foo_bool = std::make_shared<Parameter<bool>>("foo_bool", true);
        foo_int  = std::make_shared<NumericParameter<int>>("foo_int", 3, 0, 5);
        mutable_internal_param_list = {
            foo_bool,
            foo_int,
        };
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<bool>>(foo_bool),
            std::const_pointer_cast<const NumericParameter<int>>(foo_int),
        };
    }

    const std::shared_ptr<const Parameter<bool>> fooBool() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(foo_bool);
    }

    const std::shared_ptr<Parameter<bool>> mutableFooBool()
    {
        return foo_bool;
    }

    const std::shared_ptr<const NumericParameter<int>> fooInt() const
    {
        return std::const_pointer_cast<const NumericParameter<int>>(foo_int);
    }

    const std::shared_ptr<NumericParameter<int>> mutableFooInt()
    {
        return foo_int;
    }

    const std::string name() const
    {
        return "fooConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help     = false;
            bool foo_bool = true;
            int foo_int   = 3;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "help screen");

        desc.add_options()(
            "foo_bool", boost::program_options::value<bool>(&args.foo_bool), "foo bool");
        desc.add_options()("foo_int", boost::program_options::value<int>(&args.foo_int),
                           "foo int");

        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableFooBool()->setValue(args.foo_bool);
        this->mutableFooInt()->setValue(args.foo_int);

        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<bool>> foo_bool;
    std::shared_ptr<NumericParameter<int>> foo_int;
};

class exampleConfig : public Config
{
   public:
    exampleConfig(std::shared_ptr<fooConfig> foo_config) : foo_config(foo_config)
    {
        example_bool_param =
            std::make_shared<Parameter<bool>>("example_bool_param", true);
        example_int_param =
            std::make_shared<NumericParameter<int>>("example_int_param", 3, 0, 5);
        example_float_param = std::make_shared<NumericParameter<float>>(
            "example_float_param", 4.04, 1.1, 9.01);
        example_string_param = std::make_shared<Parameter<std::string>>(
            "example_string_param", "Hello World");
        example_enum_param = std::make_shared<EnumeratedParameter<std::string>>(
            "example_enum_param", "HALT", allValuesRefereeCommandString());
        example_factory_param = std::make_shared<EnumeratedParameter<std::string>>(
            "example_factory_param", "HaltPlay",
            GenericFactory<std::string, Play>::getRegisteredNames());
        mutable_internal_param_list = {
            example_bool_param,   example_int_param,  example_float_param,
            example_string_param, example_enum_param, example_factory_param,
            foo_config,
        };
        immutable_internal_param_list = {
            std::const_pointer_cast<const Parameter<bool>>(example_bool_param),
            std::const_pointer_cast<const NumericParameter<int>>(example_int_param),
            std::const_pointer_cast<const NumericParameter<float>>(example_float_param),
            std::const_pointer_cast<const Parameter<std::string>>(example_string_param),
            std::const_pointer_cast<const EnumeratedParameter<std::string>>(
                example_enum_param),
            std::const_pointer_cast<const EnumeratedParameter<std::string>>(
                example_factory_param),
            std::const_pointer_cast<const fooConfig>(foo_config),
        };
    }

    const std::shared_ptr<const Parameter<bool>> exampleBoolParam() const
    {
        return std::const_pointer_cast<const Parameter<bool>>(example_bool_param);
    }

    const std::shared_ptr<Parameter<bool>> mutableExampleBoolParam()
    {
        return example_bool_param;
    }

    const std::shared_ptr<const NumericParameter<int>> exampleIntParam() const
    {
        return std::const_pointer_cast<const NumericParameter<int>>(example_int_param);
    }

    const std::shared_ptr<NumericParameter<int>> mutableExampleIntParam()
    {
        return example_int_param;
    }

    const std::shared_ptr<const NumericParameter<float>> exampleFloatParam() const
    {
        return std::const_pointer_cast<const NumericParameter<float>>(
            example_float_param);
    }

    const std::shared_ptr<NumericParameter<float>> mutableExampleFloatParam()
    {
        return example_float_param;
    }

    const std::shared_ptr<const Parameter<std::string>> exampleStringParam() const
    {
        return std::const_pointer_cast<const Parameter<std::string>>(
            example_string_param);
    }

    const std::shared_ptr<Parameter<std::string>> mutableExampleStringParam()
    {
        return example_string_param;
    }

    const std::shared_ptr<const EnumeratedParameter<std::string>> exampleEnumParam() const
    {
        return std::const_pointer_cast<const EnumeratedParameter<std::string>>(
            example_enum_param);
    }

    const std::shared_ptr<EnumeratedParameter<std::string>> mutableExampleEnumParam()
    {
        return example_enum_param;
    }

    const std::shared_ptr<const EnumeratedParameter<std::string>> exampleFactoryParam()
        const
    {
        return std::const_pointer_cast<const EnumeratedParameter<std::string>>(
            example_factory_param);
    }

    const std::shared_ptr<EnumeratedParameter<std::string>> mutableExampleFactoryParam()
    {
        return example_factory_param;
    }

    const std::shared_ptr<const fooConfig> getFooConfig()
    {
        return std::const_pointer_cast<const fooConfig>(foo_config);
    }

    const std::shared_ptr<fooConfig> getMutableFooConfig()
    {
        return foo_config;
    }

    const std::string name() const
    {
        return "exampleConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct fooConfigCommandLineArgs
        {
            bool foo_bool = true;
            int foo_int   = 3;
        };

        struct commandLineArgs
        {
            bool help                         = false;
            bool example_bool_param           = true;
            int example_int_param             = 3;
            float example_float_param         = 4.04f;
            std::string example_string_param  = "Hello World";
            std::string example_enum_param    = "HALT";
            std::string example_factory_param = "HaltPlay";
            fooConfigCommandLineArgs foo_config_args;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "help screen");

        desc.add_options()("example_bool_param",
                           boost::program_options::value<bool>(&args.example_bool_param),
                           "Can be true or false");
        desc.add_options()("example_int_param",
                           boost::program_options::value<int>(&args.example_int_param),
                           "Can be any integer value in the range [min, max]");
        desc.add_options()(
            "example_float_param",
            boost::program_options::value<float>(&args.example_float_param),
            "Can be any value in the range [min, max] For C code, this will be generated as a float. For C++ code, this will be generated as a double.");
        desc.add_options()(
            "example_string_param",
            boost::program_options::value<std::string>(&args.example_string_param),
            "Any string value");
        desc.add_options()(
            "example_enum_param",
            boost::program_options::value<std::string>(&args.example_enum_param),
            "Accepts enum values from the specified enum");
        desc.add_options()(
            "example_factory_param",
            boost::program_options::value<std::string>(&args.example_factory_param),
            "Accepts all registered IndexType values for the GenericFactory<index_type, type_to_create>");
        desc.add_options()(
            "fooConfig.foo_bool",
            boost::program_options::value<bool>(&args.foo_config_args.foo_bool),
            "foo bool");
        desc.add_options()(
            "fooConfig.foo_int",
            boost::program_options::value<int>(&args.foo_config_args.foo_int), "foo int");

        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        this->mutableExampleBoolParam()->setValue(args.example_bool_param);
        this->mutableExampleIntParam()->setValue(args.example_int_param);
        this->mutableExampleFloatParam()->setValue(args.example_float_param);
        this->mutableExampleStringParam()->setValue(args.example_string_param);
        this->mutableExampleEnumParam()->setValue(args.example_enum_param);
        this->mutableExampleFactoryParam()->setValue(args.example_factory_param);
        this->getMutableFooConfig()->mutableFooBool()->setValue(
            args.foo_config_args.foo_bool);
        this->getMutableFooConfig()->mutableFooInt()->setValue(
            args.foo_config_args.foo_int);

        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    std::vector<std::string> allValuesRefereeCommandString()
    {
        return separateEnumStrings(std::string{enum_string_args_RefereeCommand});
    }

    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<Parameter<bool>> example_bool_param;
    std::shared_ptr<NumericParameter<int>> example_int_param;
    std::shared_ptr<NumericParameter<float>> example_float_param;
    std::shared_ptr<Parameter<std::string>> example_string_param;
    std::shared_ptr<EnumeratedParameter<std::string>> example_enum_param;
    std::shared_ptr<EnumeratedParameter<std::string>> example_factory_param;
    std::shared_ptr<fooConfig> foo_config;
};

class ThunderbotsConfig : public Config
{
   public:
    ThunderbotsConfig()
    {
        foo_config                    = std::make_shared<fooConfig>();
        example_config                = std::make_shared<exampleConfig>(foo_config);
        mutable_internal_param_list   = {foo_config, example_config};
        immutable_internal_param_list = {
            std::const_pointer_cast<const fooConfig>(foo_config),
            std::const_pointer_cast<const exampleConfig>(example_config)};
    }

    const std::shared_ptr<const fooConfig> getFooConfig() const
    {
        return std::const_pointer_cast<const fooConfig>(foo_config);
    }

    const std::shared_ptr<fooConfig> getMutableFooConfig()
    {
        return foo_config;
    }

    const std::shared_ptr<const exampleConfig> getExampleConfig() const
    {
        return std::const_pointer_cast<const exampleConfig>(example_config);
    }

    const std::shared_ptr<exampleConfig> getMutableExampleConfig()
    {
        return example_config;
    }

    const std::string name() const
    {
        return "ThunderbotsConfig";
    }

    bool loadFromCommandLineArguments(int argc, char** argv)
    {
        struct commandLineArgs
        {
            bool help = false;
        };

        commandLineArgs args;
        boost::program_options::options_description desc{"Options"};

        desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                           "Help screen");

        boost::program_options::variables_map vm;
        boost::program_options::store(parse_command_line(argc, argv, desc), vm);
        boost::program_options::notify(vm);

        if (args.help)
        {
            std::cout << desc << std::endl;
        }

        return args.help;
    }

    const MutableParameterList& getMutableParameterList()
    {
        return mutable_internal_param_list;
    }

    const ParameterList& getParameterList() const
    {
        return immutable_internal_param_list;
    }

   private:
    MutableParameterList mutable_internal_param_list;
    ParameterList immutable_internal_param_list;
    std::shared_ptr<exampleConfig> example_config;
    std::shared_ptr<fooConfig> foo_config;
};
