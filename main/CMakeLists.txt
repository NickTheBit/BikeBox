
set(Sources "")
list(APPEND Sources "main.cpp")

set(SubDirs "")
list(APPEND SubDirs ".")
list(APPEND SubDirs "Recorder")
list(APPEND SubDirs "System")
list(APPEND SubDirs "UserInterface")
list(APPEND SubDirs "SegDisplay")

idf_component_register(
        SRC_DIRS ${SubDirs}
        INCLUDE_DIRS ${SubDirs}
)
