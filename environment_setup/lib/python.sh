# Virtual environment management.
#
# Bazel and the runtime scripts expect the interpreter at a fixed path under
# /opt/tbotspython, so that is what every module installs into.

create_tbotspython_venv() {
  local args=()
  read -r -a args <<<"$(venv_args)"

  # The directory is recreated from scratch so that packages removed from
  # requirements.txt do not linger across runs.
  as_root rm -rf "$VENV_DIR"
  as_root "$(python_interpreter)" -m venv "$VENV_DIR" "${args[@]}"
  as_root "$(venv_bin pip)" install --upgrade pip

  # Install into the venv's own pip rather than whatever is on PATH.
  as_root "$(venv_bin pip)" install -r "$SETUP_DIR/requirements.txt"

  # Hand the directory back to the user so that later steps do not need sudo.
  as_root chown -R "$(id -u):$(id -g)" "$VENV_DIR"
}

prepare_external_runtimes() {
  as_root mkdir -p "$VENV_DIR/external_runtimes"
  as_root chown -R "$(id -u):$(id -g)" "$VENV_DIR/external_runtimes"
}
