# ##############################################################################
# apps/examples/encoder/CMakeLists.txt
#
# SPDX-License-Identifier: Apache-2.0
# ##############################################################################

if(CONFIG_EXAMPLES_ENCODER)
  nuttx_add_application(
    NAME
    ${CONFIG_EXAMPLES_ENCODER_PROGNAME}
    PRIORITY
    ${CONFIG_EXAMPLES_ENCODER_PRIORITY}
    STACKSIZE
    ${CONFIG_EXAMPLES_ENCODER_STACKSIZE}
    MODULE
    ${CONFIG_EXAMPLES_ENCODER}
    SRCS
    encoder_main.c
    position_sensor.c)
endif()

