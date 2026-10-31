cmake_minimum_required(VERSION 3.22)

# Wspolne komponenty dzielone przez wszystkie trzy jednostki. Musi byc
# ustawione PRZED include(project.cmake), inaczej nie zostana znalezione.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(tagcam)

# Port AprilTag nie buduje sie na GCC 15 z ESP-IDF 6.1: w apriltag/common/
# getopt.c sa dwa niezgodne typy wskaznikow, a ten diagnostyk przestal byc
# ostrzezeniem i stal sie bledem. Komponentowe -w tego nie tlumi.
#
# getopt.c to parser argumentow wiersza polecen - na mikrokontrolerze kod
# martwy, wiec lagodzimy diagnostyk WYLACZNIE dla tego komponentu. Nasz kod
# dalej traktuje go jako blad, i tak ma zostac.
#
# Do usuniecia, gdy komponent zostanie poprawiony u zrodla.
idf_component_get_property(apriltag_lib liux-pro__apriltag COMPONENT_LIB)
#
# Drugi problem: biblioteka uzywa starego zapisu wskaznikow na funkcje,
# `void (*f)()`, ktory w C23 znaczy "zero argumentow" zamiast
# "nieokreslone". GCC 15 domyslnie kompiluje w C23, wiec zarray, zhash
# i zmaxheap przestaly sie budowac. Nie jest to blad biblioteki, tylko
# zmiana standardu - kompilujemy ja wiec standardem, pod ktory powstala.
target_compile_options(${apriltag_lib} PRIVATE
                       -std=gnu17
                       -Wno-error=incompatible-pointer-types)
