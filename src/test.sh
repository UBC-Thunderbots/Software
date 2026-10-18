source /opt/tbots/bin/activate

bazel test --copt=-O3 --flaky_test_attempts=3 --show_timestamps \
  --test_arg="--ci_mode"                                        \
  -- //software:unix_full_system                                \
  -//software/gameplay_tests:requirements.test                  \
  //software/ai/stp/...                                         \
  //software/ai/navigator/...                                   \
  //software/simulation/...

