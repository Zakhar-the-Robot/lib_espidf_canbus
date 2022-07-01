idf_component_register( SRCS 
                        "${CMAKE_CURRENT_LIST_DIR}/src/canbus.cpp"
                        "${CMAKE_CURRENT_LIST_DIR}/src/delay.c"
                        "${CMAKE_CURRENT_LIST_DIR}/src/millis.c"
                        
                        INCLUDE_DIRS 
                        "${CMAKE_CURRENT_LIST_DIR}/include"
                        "${CMAKE_CURRENT_LIST_DIR}/src/include"
                        REQUIRES hal )
