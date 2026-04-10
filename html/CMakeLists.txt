# Automatically embed (gzip compressed) assets into the project

# glob all *.html, *.css and *.js files for embedding
file(GLOB files "${project_dir}/html/*.html" "${project_dir}/html/*.css" "${project_dir}/html/*.js")

foreach(file ${files})
    get_filename_component(filename ${file} NAME)
    set(gz_file "${CMAKE_CURRENT_BINARY_DIR}/compressed/${filename}.gz")
    
    # make sure the output directory exists
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/compressed")

    if(CONFIG_WEBSERVER_USE_GZIP)
        message(STATUS "Add asset compressed: ${filename}")

        # add command to generate .gz files for embedding
        add_custom_command(
            OUTPUT "${gz_file}"
            COMMAND python ${project_dir}/html/compress.py ${file} "${gz_file}"
            DEPENDS ${file} ${project_dir}/html/compress.py
            COMMENT "Compressing: ${filename}"
            VERBATIM
        )

        # add file.ext.gz as BINARY file to ELF (like EMBED_FILES)
        target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${gz_file}" BINARY)

        # option to clean up generated .gz files when running 'make clean'
        set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES "${gz_file}")

    else()
        # add asset as plain TEXT file to binary
        target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "${file}" TEXT)

    endif()
endforeach()

