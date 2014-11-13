solution "tilemover2d"
    configurations { "Debug", "Release" }
    
    project "test"
        kind "ConsoleApp"
        language "C++"
        files {
            "../src/**.h",
            "../src/**.cpp"
            }

        if not os.is("windows") then
            buildoptions { "-std=c++11 -Wno-error=unused-variable -Wno-error=unused-parameter" }
        end
