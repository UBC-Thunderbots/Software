# Description:
#   Launches the prebuilt TIGERs AutoReferee application.
#   https://github.com/TIGERs-Mannheim/AutoReferee
#

load("@rules_java//java:java_binary.bzl", "java_binary")

java_binary(
    name = "autoref_java",
    main_class = "edu.tigers.autoref.AutoReferee",
    jvm_flags = [
        # Preserve the default JVM options from the upstream launch script.
        "-Dsun.java2d.d3d=false",
        "-Djava.net.preferIPv4Stack=true",
    ],
    runtime_deps = [
        # Includes AutoRef's application JAR and bundled dependency JARs.
        "@tigers_autoref//:jars",
    ],
    data = [
        # Make the upstream configuration and other runtime files available.
        "@tigers_autoref//:distribution",
    ],
    visibility = ["//visibility:public"],
)
