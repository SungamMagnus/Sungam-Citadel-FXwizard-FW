#
# MIT License
# Copyright (c) 2025 Vaclav Mach (Bastl Instruments)
#

# App definition
create_kastle2_app(
    APP_NAME "fx-wizard-plus"
    APP_USB_NAME "Kastle 2 FX Wizard Plus"
    APP_USB_PREFIX "K2FP_"
    APP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/AppFxWizardPlus.cpp
)