

# the instalation script place
set(_ARENA_SDK_CONF "/etc/ld.so.conf.d/Arena_SDK.conf")

if(EXISTS ${_ARENA_SDK_CONF})

	# get first line in Arena_SDK.conf which is the lib64 path. the get the
	# parent direcotry of the first path which suppose to be the location of
	# the installed ArenaSDK
	execute_process(
		COMMAND bash -c "dirname $(head -n 1 \"/etc/ld.so.conf.d/Arena_SDK.conf\")"
		OUTPUT_VARIABLE _ArenaSDK_ROOT
		ENCODING UTF8
		)
	string(STRIP ${_ArenaSDK_ROOT} _ArenaSDK_ROOT)


	set(arena_sdk_INCLUDE_DIRS
		${_ArenaSDK_ROOT}/GenICam/library/CPP/include
		${_ArenaSDK_ROOT}/include/Arena)
	set (arena_sdk_INCLUDES ${arena_sdk_INCLUDE_DIRS})


	set(arena_sdk_LIBS

		## ArenaSDK
		${_ArenaSDK_ROOT}/lib64/libsave.so
		${_ArenaSDK_ROOT}/lib64/libsaved.so
		${_ArenaSDK_ROOT}/lib64/libarena.so
		${_ArenaSDK_ROOT}/lib64/libgentld.so
		${_ArenaSDK_ROOT}/lib64/libgentl.so
		${_ArenaSDK_ROOT}/lib64/libarenad.so

		## GenICam
		${_ArenaSDK_ROOT}/GenICam/library/lib/Linux64_x64/libGCBase_gcc421_v3_0.so
		${_ArenaSDK_ROOT}/GenICam/library/lib/Linux64_x64/libGenApi_gcc421_v3_0.so
		#${_ArenaSDK_ROOT}/GenICam/library/lib/Linux64_x64/liblog4cpp_gcc421_v3_0.so
		#${_ArenaSDK_ROOT}/GenICam/library/lib/Linux64_x64/libLog_gcc421_v3_0.so
		#${_ArenaSDK_ROOT}/GenICam/library/lib/Linux64_x64/libMathParser_gcc421_v3_0.so
		#${_ArenaSDK_ROOT}/GenICam/library/lib/Linux64_x64/libNodeMapData_gcc421_v3_0.so
		#${_ArenaSDK_ROOT}/GenICam/library/lib/Linux64_x64/libXmlParser_gcc421_v3_0.so
		## fmpeg
		#${_ArenaSDK_ROOT}/ffmpeg/libavcodec.so
		#${_ArenaSDK_ROOT}/ffmpeg/libavformat.so
		#${_ArenaSDK_ROOT}/ffmpeg/libavutil.so
		#${_ArenaSDK_ROOT}/ffmpeg/libswresample.so
		)

	set(arena_sdk_LIBRARIES arena_sdk_LIBS)

	#set(arena_sdk_DEFINITIONS GENICAM_USER_ACCEPTS_ANY_COMPILER)
	
	set(arena_sdk_FOUND true)


else()
	message( FATAL_ERROR "ArenaSDK is not installed. Please isntall ArenaSDK "
						 "using the script provided by LUCID support "
						 "team (support@thinklucid.com). ")
endif()

