set(Modules "")

list(APPEND Modules .)
list(APPEND Modules mpu6050)
list(APPEND Modules Recorder)
list(APPEND Modules System)
list(APPEND Modules UserInterface)

idf_component_register(
    INCLUDE_DIRS ${Modules}
    SRC_DIRS ${Modules}
)
