
set(Sources "")
list(APPEND Sources "main.cpp")

set(SubDirs "")
list(APPEND SubDirs ".")
list(APPEND SubDirs "mpu6050")
list(APPEND SubDirs "Recorder")
list(APPEND SubDirs "System")
list(APPEND SubDirs "UserInterface")

idf_component_register(
        SRC_DIRS ${SubDirs}
        INCLUDE_DIRS ${SubDirs}
)
