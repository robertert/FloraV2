cmake_minimum_required(VERSION 3.22)

# Wspolne komponenty dzielone przez wszystkie trzy jednostki. Musi byc
# ustawione PRZED include(project.cmake), inaczej nie zostana znalezione.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tagcam)
