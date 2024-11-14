file(GLOB ALL_SCRS "*.cpp")

idf_component_register(SRCS ${ALL_SCRS}
                    INCLUDE_DIRS "include"
                    REQUIRES driver)
