check_os_and_arch() {
	OS="$(uname -s)"   # Darwin or Linux
	ARCH="$(uname -m)" # x86_64, arm64, or aarch64

	case "$OS" in
		Darwin|Linux) ;;
		*)
			fail "Error: Unsupported operating system '$OS'."
			;;
	esac

	# Normalize Architecture representation across macOS and Linux
	case "$ARCH" in
		x86_64|amd64) ARCH_NORM="x86_64" ;;
		arm64|aarch64) ARCH_NORM="arm64" ;;
		*)
			fail "Error: Unsupported architecture '$ARCH'."
			;;
	esac
}
