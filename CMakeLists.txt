idf_component_register( SRCS 
                        "${CMAKE_CURRENT_LIST_DIR}/src/canbus.cpp"
                        "${CMAKE_CURRENT_LIST_DIR}/src/delay.c"
                        "${CMAKE_CURRENT_LIST_DIR}/src/millis.c"
                        "${CMAKE_CURRENT_LIST_DIR}/src/portable/src"
                        "${CMAKE_CURRENT_LIST_DIR}/src/portable/log.cx/src/log.c"
                        
                        INCLUDE_DIRS 
                        "${CMAKE_CURRENT_LIST_DIR}/src"
                        "${CMAKE_CURRENT_LIST_DIR}/src/portable/log.cx/src"
                        
                        REQUIRES hal )
