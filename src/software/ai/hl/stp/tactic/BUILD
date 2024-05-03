package(default_visibility = ["//visibility:public"])

cc_test(
    name = "penalty_setup_tactic_test",
    srcs = ["penalty_setup_tactic_test.cpp"],
    deps = [
        "//shared/test_util:tbots_gtest_main",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/test_util",
    ],
)

cc_library(
    name = "all_tactics",
    hdrs = ["all_tactics.h"],
    deps = [
        ":tactic",
        "//software/ai/hl/stp/tactic/assigned_skill:assigned_skill_tactic",
        "//software/ai/hl/stp/tactic/assigned_skill:assigned_skill_tactics",
        "//software/ai/hl/stp/tactic/attacker:attacker_tactic",
        "//software/ai/hl/stp/tactic/crease_defender:crease_defender_tactic",
        "//software/ai/hl/stp/tactic/goalie:goalie_tactic",
        "//software/ai/hl/stp/tactic/move:move_tactic",
        "//software/ai/hl/stp/tactic/pass_defender:pass_defender_tactic",
        "//software/ai/hl/stp/tactic/penalty_kick:penalty_kick_tactic",
        "//software/ai/hl/stp/tactic/receiver:receiver_tactic",
        "//software/ai/hl/stp/tactic/shadow_enemy:shadow_enemy_tactic",
        "//software/ai/hl/stp/tactic/stop:stop_tactic",
    ],
)

cc_library(
    name = "tactic",
    srcs = ["tactic.cpp"],
    hdrs = [
        "tactic.h",
        "tactic_visitor.h",
    ],
    deps = [
        ":tactic_fsm",
        ":transition_conditions",
        "//software/ai/hl/stp/primitive:stop_primitive",
        "//software/util/typename",
        "//software/world",
        "@tracy",
    ],
)

cc_library(
    name = "tactic_fsm",
    hdrs = ["tactic_fsm.h"],
    deps = [
        "//proto/primitive:primitive_msg_factory",
        "//software/util/sml_fsm",
        "//software/util/typename",
        "//software/world",
        "@tracy",
    ],
)

cc_library(
    name = "transition_conditions",
    srcs = ["transition_conditions.cpp"],
    hdrs = ["transition_conditions.h"],
    deps = [
        "//software/world",
    ],
)

cc_test(
    name = "transition_conditions_test",
    srcs = ["transition_conditions_test.cpp"],
    deps = [
        ":tactic",
        "//shared/test_util:tbots_gtest_main",
        "//software/world",
    ],
)

cc_library(
    name = "tactic_factory",
    srcs = ["tactic_factory.cpp"],
    hdrs = [
        "tactic_factory.h",
    ],
    deps = [
        ":all_tactics",
        "//proto:tbots_cc_proto",
        "//proto/message_translation:tbots_protobuf",
        "//software/ai/hl/stp/strategy",
    ],
)
