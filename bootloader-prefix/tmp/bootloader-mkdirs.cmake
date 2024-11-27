# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Software/ESP-IDF_Extension/ESP-IDF_V5.0.3/esp-idf/components/bootloader/subproject"
  "C:/Users/phoen/Dropbox/Git/GitHub/ESP32S3_Adapter_ESP_IDF/bootloader"
  "C:/Users/phoen/Dropbox/Git/GitHub/ESP32S3_Adapter_ESP_IDF/bootloader-prefix"
  "C:/Users/phoen/Dropbox/Git/GitHub/ESP32S3_Adapter_ESP_IDF/bootloader-prefix/tmp"
  "C:/Users/phoen/Dropbox/Git/GitHub/ESP32S3_Adapter_ESP_IDF/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/phoen/Dropbox/Git/GitHub/ESP32S3_Adapter_ESP_IDF/bootloader-prefix/src"
  "C:/Users/phoen/Dropbox/Git/GitHub/ESP32S3_Adapter_ESP_IDF/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/phoen/Dropbox/Git/GitHub/ESP32S3_Adapter_ESP_IDF/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/phoen/Dropbox/Git/GitHub/ESP32S3_Adapter_ESP_IDF/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
