cmake_minimum_required(VERSION 3.5)

# set the project name
project(WWD_Engine)

# add_compile_options("-fopenmp")
# add_compile_options("-Ofast")
#add_compile_options("-Werror")
#add_compile_options("-g")

#add_subdirectory(lib)


add_executable(WWD_Engine main.c)
target_link_libraries(WWD_Engine PUBLIC ${PROJECT_SOURCE_DIR}/lib/libWWDLib.a)
target_link_libraries(WWD_Engine PUBLIC portaudio)
target_link_libraries(WWD_Engine PUBLIC m)
target_link_libraries(WWD_Engine PUBLIC pthread)
target_link_libraries(WWD_Engine PUBLIC asound)

target_include_directories(WWD_Engine PUBLIC
                          ${PROJECT_BINARY_DIR}
                          ${PROJECT_SOURCE_DIR}/include/
                          )