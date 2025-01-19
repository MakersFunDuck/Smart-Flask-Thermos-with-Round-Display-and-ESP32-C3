SET(SOURCES screens/ui_Screen1.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_gradient_png.c
    fonts/ui_font_monoton70.c
    fonts/ui_font_monoton90.c)

add_library(ui ${SOURCES})
