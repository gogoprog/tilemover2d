solution "tilemover2d"
    configurations { "Debug", "Release" }
    
    project "test"
        kind "ConsoleApp"
        language "C++"
        files {
            "../src/**.h",
            "../src/**.cpp"
            }

