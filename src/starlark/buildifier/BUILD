load("@buildifier_prebuilt//:rules.bzl", "buildifier")

buildifier(
    name = "buildifier.fix",
    exclude_patterns = [
        "./.git/*",
    ],
    lint_mode = "warn",
    mode = "fix",
)
