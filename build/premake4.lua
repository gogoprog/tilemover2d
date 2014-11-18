solution "tilemover2d"
    configurations { "Debug" }
    
    platforms { "x32", "x64" }

    project "tilemover2d-test"
        kind "ConsoleApp"
        language "C++"
        files {
            "../src/**.h",
            "../src/**.cpp"
            }
        links {
            "GL",
            "GLU",
            "glut"
        }
        defines { "DEBUG" }
        flags { "Symbols" }

        if not os.is("windows") then
            buildoptions { "-std=c++11 -Wno-error=unused-variable -Wno-error=unused-parameter" }
        end
