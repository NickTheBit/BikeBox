set(Sources "")

list(APPEND Sources Src/main.cpp)
list(APPEND Sources Src/system.cpp)
list(APPEND Sources Src/userInterface.cpp)
list(APPEND Sources Src/recorder.cpp)
list(APPEND Sources Src/mpu6050/i2c_interface.cpp)
list(APPEND Sources Src/mpu6050/mpu6050.cpp)

idf_component_register(SRCS ${Sources}
                    INCLUDE_DIRS ".")

include_directories("Inc")
