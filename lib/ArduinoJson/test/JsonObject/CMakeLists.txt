# ArduinoJson - arduinojson.org
# Copyright Benoit Blanchon 2014-2023
# MIT License

add_executable(JsonObjectTests
	basics.cpp
	containsKey.cpp
	createNestedArray.cpp
	createNestedObject.cpp
	get.cpp
	invalid.cpp
	iterator.cpp
	prettyPrintTo.cpp
	printTo.cpp
	remove.cpp
	set.cpp
	size.cpp
	subscript.cpp
)

target_link_libraries(JsonObjectTests catch)
add_test(JsonObject JsonObjectTests)
