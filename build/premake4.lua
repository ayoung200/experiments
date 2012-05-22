solution "0MySolution"

	-- Multithreaded compiling
	if _ACTION == "vs2010" or _ACTION=="vs2008" then
		buildoptions { "/MP"  }
	elseif _ACTION == "gmake" then
		buildoptions {"-fexceptions -fpermissive -std=c++0x"}
	end 
	
	newoption {
    trigger     = "with-nacl",
    description = "Enable Native Client build"
  }
  
  newoption {
    trigger     = "with-pe",
    description = "Enable Physics Effects"
  }
  
	configurations {"Release", "Debug"}
	configuration "Release"
		flags { "Optimize", "StaticRuntime", "NoMinimalRebuild", "FloatFast"}
	configuration "Debug"
		defines {"_DEBUG=1"}
		flags { "Symbols", "StaticRuntime" , "NoMinimalRebuild", "NoEditAndContinue" ,"FloatFast"}
		
	platforms {"x32", "x64"}

	configuration {"x32"}
		targetsuffix ("_" .. _ACTION)
	configuration "x64"		
		targetsuffix ("_" .. _ACTION .. "_64" )
	configuration {"x64", "debug"}
		targetsuffix ("_" .. _ACTION .. "_x64_debug")
	configuration {"x64", "release"}
		targetsuffix ("_" .. _ACTION .. "_x64_release" )
	configuration {"x32", "debug"}
		targetsuffix ("_" .. _ACTION .. "_debug" )
	
	configuration{}

if not _OPTIONS["with-nacl"] then
	--	flags { "NoRTTI", "NoExceptions"}
	--	defines { "_HAS_EXCEPTIONS=0" }
		targetdir "../bin"
	  location("./" .. _ACTION)

else
--	targetdir "../bin_html"

--remove some default flags when cross-compiling for Native Client
--see also http://industriousone.com/topic/how-remove-usrlib64-x86-builds-cross-compiling
	premake.gcc.platforms.x64.ldflags = string.gsub(premake.gcc.platforms.x64.ldflags, "-L/usr/lib64", "")
	premake.gcc.platforms.x32.ldflags = string.gsub(premake.gcc.platforms.x32.ldflags, "-L/usr/lib32", "")
	
	targetdir "nacl/nginx-1.1.2/html"
	
	location("./nacl")
end

	

	projectRootDir = os.getcwd() .. "/../"
	print("Project root directroy: " .. projectRootDir);

	dofile ("findOpenCL.lua")
	dofile ("findDirectX11.lua")
	dofile ("findOpenGLGlewGlut.lua")
	
	language "C++"
	

	include "../bullet2"	
	include "../jpeglib"

	

	
if not _OPTIONS["with-nacl"] then

	include "../opencl/c_api"
	include "../opencl/basic_initialize"
	include "../opencl/vector_add"
	include "../opencl/gui_initialize"
--	include "../opencl/opengl_interop"
	include "../opencl/global_atomics"
--	include "../opencl/integration"

	include "../opencl/benchmark/sort"
--	include "../opencl/primitives/benchmark"
	include "../rendering/GLSL_Instancing"
	include "../opencl/3dGridBroadphase"
	include "../opencl/broadphase_benchmark"
	--include "../opencl/gpu_rigidbody_pipeline"
	--include "../opencl/gpu_rigidbody_pipeline_sph"
	include "../opencl/gpu_rigidbody_pipeline2"
	
	include "../dynamics/profiler_test"
	--include "../Lua"
	
	
if _OPTIONS["with-pe"] then
	
	include "../physics_effects/base_level"
	include "../physics_effects/low_level"
	include "../physics_effects/util"
	include "../physics_effects/sample_api_physics_effects/0_console"
	
	include "../physics_effects/sample_api_physics_effects/1_simple"
	include "../physics_effects/sample_api_physics_effects/2_stable"
	include "../physics_effects/sample_api_physics_effects/3_sleep"
	include "../physics_effects/sample_api_physics_effects/4_motion_type"
	include "../physics_effects/sample_api_physics_effects/5_raycast"
	include "../physics_effects/sample_api_physics_effects/6_joint"

end
	
	include "../dynamics/testbed"
	include "../dynamics/position_based_dynamics"
--	include "../dynamics/basic_demo"
	

	include "../dynamics/exact-ccd"
--	include "../dynamics/corotational_fem"
	--include "../dynamics/nncg_test"

	include "../rendering/Gwen/Gwen"
	include "../rendering/Gwen/GwenOpenGLTest"
	include "../rendering/OpenGLES2Angle"
else
	include "../rendering/NativeClient"	
	
end
