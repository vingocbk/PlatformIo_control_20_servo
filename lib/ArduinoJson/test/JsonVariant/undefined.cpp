// ArduinoJson - arduinojson.org
// Copyright Benoit Blanchon 2014-2023
// MIT License

#include <ArduinoJson.h>
#include <catch.hpp>

TEST_CASE("JsonVariant undefined") {
  JsonVariant variant;

  SECTION("AsLongReturns0") {
    REQUIRE(0 == variant.as<long>());
  }

  SECTION("AsUnsignedReturns0") {
    REQUIRE(0 == variant.as<unsigned>());
  }

  SECTION("AsStringReturnsNull") {
    REQUIRE(0 == variant.as<char*>());
  }

  SECTION("AsDoubleReturns0") {
    REQUIRE(0 == variant.as<double>());
  }

  SECTION("AsBoolReturnsFalse") {
    REQUIRE(false == variant.as<bool>());
  }

  SECTION("AsArrayReturnInvalid") {
    REQUIRE(JsonArray::invalid() == variant.as<JsonArray&>());
  }

  SECTION("AsConstArrayReturnInvalid") {
    REQUIRE(JsonArray::invalid() == variant.as<const JsonArray&>());
  }

  SECTION("AsObjectReturnInvalid") {
    REQUIRE(JsonObject::invalid() == variant.as<JsonObject&>());
  }

  SECTION("AsConstObjectReturnInvalid") {
    REQUIRE(JsonObject::invalid() == variant.as<const JsonObject&>());
  }

  SECTION("AsArrayWrapperReturnInvalid") {
    REQUIRE(JsonArray::invalid() == variant.as<JsonArray>());
  }

  SECTION("AsObjectWrapperReturnInvalid") {
    REQUIRE(JsonObject::invalid() == variant.as<JsonObject>());
  }
}
