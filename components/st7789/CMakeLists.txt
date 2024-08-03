set(srcs "st7789.c")

idf_component_register(SRCS "jcfx.c" "${srcs}"
                       PRIV_REQUIRES driver
                       INCLUDE_DIRS ".")
