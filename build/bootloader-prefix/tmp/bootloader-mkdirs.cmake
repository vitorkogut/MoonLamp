# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/vitor/esp/esp-idf/components/bootloader/subproject"
  "C:/fast_scan/build/bootloader"
  "C:/fast_scan/build/bootloader-prefix"
  "C:/fast_scan/build/bootloader-prefix/tmp"
  "C:/fast_scan/build/bootloader-prefix/src/bootloader-stamp"
  "C:/fast_scan/build/bootloader-prefix/src"
  "C:/fast_scan/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/fast_scan/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/fast_scan/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
