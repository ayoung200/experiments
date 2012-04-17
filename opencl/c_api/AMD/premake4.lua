	
	hasCL = findOpenCL_AMD()
	
	if (hasCL) then

		project "OpenCL_C_API_Test"

		initOpenCL_AMD()
	
		kind "ConsoleApp"
		targetdir "../../../bin"

		includedirs 
		{
			projectRootDir .. "bullet2",
			projectRootDir .. "opencl/primitives"
		}
		
		links {"bullet2"}

		language "C"
		files {
			"../main.c",
		}

		language "C++"
		files {
			"../../basic_initialize/btOpenCLUtils.cpp",
			"../../basic_initialize/btOpenCLUtils.h",
			"../btbDeviceCL.cpp",
			"../btbDeviceCL.h",
			"../btbPlatformDefinitions.h",
			"../btcFindPairs.cpp",
			"../btcFindPairs.h",
			"../Test_FindPairs.cpp",
			"../Test_FindPairs.h"
		}
		
	end