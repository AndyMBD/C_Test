function(create_vsc_launch_json EXE_NAME)

    set(LAUNCH_JSON_CONTENT) # explicit init, this string will be used in the end to configure the launch.json template file

    set(jsonMainTemplateString
[==[
        {
            "name": "Debug sample1_unittest",
            "type": "cppdbg",
            "request": "launch",
            "program": "@CMAKE_CURRENT_BINARY_DIR@/@EXE_NAME@@CMAKE_EXECUTABLE_SUFFIX@",
            "args": [],
            "stopAtEntry": true,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "miDebuggerPath": "C:/msys64/ucrt64/bin/gdb.exe",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                },
                {
                    "description": "Set Disassembly Flavor to Intel",
                    "text": "-gdb-set disassembly-flavor intel",
                    "ignoreFailures": true
                }
            ]          
        },
]==])

string(CONFIGURE ${jsonMainTemplateString} tempString @ONLY)
string(APPEND LAUNCH_JSON_CONTENT ${tempString})

#     ######################################################################################
#     # Create actual launch.json file
#     ######################################################################################

    set(configureFileName launch.json)
    # message(STATUS "CMAKE_SOURCE_DIR  ${CMAKE_SOURCE_DIR}/misc/dbg_vsc/${configureFileName}")
    # message(STATUS "CMAKE_SOURCE_DIR  ${CMAKE_SOURCE_DIR}/.vscode/${configureFileName}")
    set(configureFileSrc  ${CMAKE_SOURCE_DIR}/misc/dbg_vsc/${configureFileName})
    set(configureFileDst  ${CMAKE_SOURCE_DIR}/.vscode/${configureFileName})    

    message(STATUS "Creating Visual Studio Code debugger launch configuration")
    message(STATUS "  Writing file ${configureFileDst}")
    configure_file(${configureFileSrc} ${configureFileDst} @ONLY)

endfunction()

