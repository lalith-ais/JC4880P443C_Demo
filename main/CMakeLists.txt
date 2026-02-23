idf_component_register(SRCS "jc4880p443c_demo.c"
                    INCLUDE_DIRS "."
                    REQUIRES 
                        jc4880p443c
                        lvgl
                        esp_lcd_st7701
						esp_timer
)
