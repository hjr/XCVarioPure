
file(GLOB SUB_SRCS "*.cpp")

target_sources(${COMPONENT_LIB} PRIVATE ${SUB_SRCS})
target_include_directories(${COMPONENT_LIB} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# Add optimization flags only for math sources
# target_compile_options(${COMPONENT_LIB} PRIVATE
#     $<$<COMPILE_LANGUAGE:CXX>:-O2>
#     $<$<COMPILE_LANGUAGE:C>:-O2>
# )