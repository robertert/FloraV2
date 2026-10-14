cmake_minimum_required(VERSION 3.22)

# Wspolne komponenty dzielone przez node i hub. Musi byc ustawione PRZED
# include(project.cmake), inaczej komponenty nie zostana znalezione.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(node)
