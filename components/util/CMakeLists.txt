idf_component_register(SRC_DIRS "math"
                       INCLUDE_DIRS "." 
                       REQUIRES 
)

# explicit set C++17 standard for this component
set_target_properties(${COMPONENT_LIB} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# using <cmath> etc.
target_link_libraries(${COMPONENT_LIB} PRIVATE m)