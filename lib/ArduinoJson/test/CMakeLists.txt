# ArduinoJson - arduinojson.org
# Copyright Benoit Blanchon 2014-2023
# MIT License

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
	add_compile_options(
		-pedantic
		-Wall
		-Wcast-align
		-Wcast-qual
		-Wconversion
		-Wctor-dtor-privacy
		-Wdisabled-optimization
		-Werror
		-Wextra
		-Wformat=2
		-Winit-self
		-Wmissing-include-dirs
		-Wnon-virtual-dtor
		-Wold-style-cast
		-Woverloaded-virtual
		-Wparentheses
		-Wredundant-decls
		-Wshadow
		-Wsign-promo
		-Wstrict-aliasing
		-Wundef
	)

	if(NOT MINGW)
		add_compile_options(
			-std=c++98
		)
	endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
	add_compile_options(
		-Wstrict-null-sentinel
	)

	if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.5)
		add_compile_options(-Wlogical-op) # the flag exists in 4.4 but is buggy
	endif()

	if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.6)
		add_compile_options(-Wnoexcept)
	endif()

	if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.7 AND
	   CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.8)
		# avoid false positive with GCC 4.7
		add_compile_options(-Wno-maybe-uninitialized)
	endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(
		-Wc++11-compat
		-Wdeprecated-register
	)
endif()

if(MSVC)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
	add_compile_options(
		/W4 # Set warning level
		/WX # Treats all compiler warnings as errors.
	)
endif()

add_subdirectory(DynamicJsonBuffer)
add_subdirectory(IntegrationTests)
add_subdirectory(JsonArray)
add_subdirectory(JsonBuffer)
add_subdirectory(JsonObject)
add_subdirectory(JsonVariant)
add_subdirectory(JsonWriter)
add_subdirectory(Misc)
add_subdirectory(Polyfills)
add_subdirectory(StaticJsonBuffer)
