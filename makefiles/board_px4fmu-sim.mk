#
# Board-specific definitions for the PX4FMUv2
#

#
# Configure the toolchain
#
CONFIG_ARCH			 = NATIVE
CONFIG_BOARD			 = PX4FMU_SIM

include $(PX4_MK_DIR)/toolchain_clang.mk
