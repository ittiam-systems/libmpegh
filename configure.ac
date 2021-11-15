AC_INIT(MPEGHDecoder, 1.0)
AC_CONFIG_MACRO_DIR([m4])
AM_INIT_AUTOMAKE([foreign subdir-objects])
AM_PROG_AS
AC_PROG_CC
AM_SILENT_RULES([yes])
AC_CANONICAL_HOST
case "$host_cpu" in
	amd64|x86_64)
		cpu_x86_64=true
		AC_DEFINE(X86_64)
		;;
	x86|i686)
		cpu_x86=true
		AC_DEFINE(X86)
		;;
	aarch64)
		cpu_armv8=true
		AC_DEFINE(ARMV8)
		;;
	armv7l)
		cpu_armv7=true
		AC_DEFINE(ARMV7)
		;;
esac
		
AM_CONDITIONAL(X86_64, test "x$cpu_x86_64" = xtrue)
AM_CONDITIONAL(X86, test "x$cpu_x86" = xtrue)
AM_CONDITIONAL(ARMV7, test "x$cpu_armv7" = xtrue)
AM_CONDITIONAL(ARMV8, test "x$cpu_armv8" = xtrue)

AC_PROG_CXX
AC_PROG_LIBTOOL
AC_SUBST(LIBTOOL_DEPS)
AC_LTDL_DLLIB
AC_CONFIG_FILES([Makefile])
AC_OUTPUT
