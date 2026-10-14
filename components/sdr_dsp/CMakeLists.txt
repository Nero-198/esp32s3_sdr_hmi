set(sdr_dsp_srcs sdr_spectrum.c sdr_trace.c)

if(ESP_PLATFORM)
  idf_component_register(SRCS ${sdr_dsp_srcs} INCLUDE_DIRS include)
else()
  # Host build used by tests/host.
  add_library(sdr_dsp STATIC ${sdr_dsp_srcs})
  target_include_directories(sdr_dsp PUBLIC include)
  target_link_libraries(sdr_dsp PUBLIC m)
endif()
