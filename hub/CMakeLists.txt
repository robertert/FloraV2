cmake_minimum_required(VERSION 3.22)

# Wspolne komponenty dzielone przez node i hub. Musi byc ustawione PRZED
# include(project.cmake), inaczej komponenty nie zostana znalezione.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hub)

# Bryla nie uzywa AprilTagow, ale wspolny flora_tags wciaga port AprilTag
# z rejestru do kazdego projektu, a menedzer komponentow buduje go mimo
# COMPONENTS/EXCLUDE_COMPONENTS. Port nie kompiluje sie na GCC 15 - to samo
# obejscie co w tagcam/CMakeLists.txt, tam jest pelne uzasadnienie.
# Do usuniecia razem z tamtym.
idf_component_get_property(apriltag_lib liux-pro__apriltag COMPONENT_LIB)
target_compile_options(${apriltag_lib} PRIVATE
                       -std=gnu17
                       -Wno-error=incompatible-pointer-types)
