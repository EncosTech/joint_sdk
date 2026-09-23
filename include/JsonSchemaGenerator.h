// SPDX-FileCopyrightText: 2026 Encos
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/seq/size.hpp>
#include <boost/preprocessor/stringize.hpp>
#include <boost/preprocessor/tuple/elem.hpp>
#include <boost/preprocessor/tuple/size.hpp>
#include <boost/preprocessor/tuple/to_seq.hpp>
#include <nlohmann/json.hpp>

#include "motor/types.h"

namespace encos {

/**
 * @brief JSON Schema字段类型枚举
 */
enum class FieldType {
    String,       ///< 字符串类型
    Integer,      ///< 整数类型
    Float,        ///< 浮点数类型
    Bool,         ///< 布尔类型
    Range,        ///< 范围类型
    Complex,      ///< 复杂类型（MotorParams）
    Axis,         ///< DoubleCoupleJoint 轴类型
    Calibration,  ///< DoubleCoupleJoint 标定参数
};

/**
 * @brief 字段信息结构体
 */
struct FieldInfo {
    const char* name;         ///< 字段名称
    FieldType type;           ///< 字段类型
    bool required;            ///< 是否必需
    const char* description;  ///< 字段描述
};

/**
 * @brief Range类型的JSON Schema定义
 *
 * 用于JSON Schema的definitions部分
 */
inline const char* kRangeSchema = R"(    "Range": {
      "type": "object",
      "properties": {
        "min": {"type": "number"},
        "max": {"type": "number"}
      },
      "required": ["min", "max"]
    })";

/**
 * @brief MotorParams类型的JSON Schema定义
 *
 * 用于JSON Schema的definitions部分。
 * 电机型号 enum 列表通过 GetAllMotorModelStrings() 动态生成，自动同步。
 * @return MotorParams 的 JSON Schema definitions 片段。
 */
inline std::string GetMotorParamsSchema() {
    std::ostringstream oss;
    oss << R"(    "MotorParams": {
      "type": "object",
      "description": "Motor initialization parameters",
      "properties": {
        "adapterId": {"type": ["string", "integer"], "description": "Adapter reference: an integer selects the adapter by creation order, a string selects it by name"},
        "busId": {"type": "integer", "description": "Bus ID"},
        "slaveId": {"type": "integer", "description": "Slave ID on the network; combined with busId as slaveId << 16 | busId"},
        "motorId": {"type": "integer", "description": "Motor ID on the bus"},
        "canfd": {"type": "boolean", "description": "Enable CAN FD frames for this motor; the global switch takes precedence"},
        "initParam": {
          "oneOf": [
            {"type": "object", "description": "Auto initialization", "properties": {}, "additionalProperties": false},
            {"type": "object", "description": "Initialize by motor model", "properties": {"model": {"type": "string", "enum": [)";
    bool first = true;
    for (const char* model : GetAllMotorModelStrings()) {
        if (!first) {
            oss << ", ";
        }
        oss << "\"" << model << "\"";
        first = false;
    }
    oss << R"(]}}, "required": ["model"]},
            {"type": "object", "description": "Initialize with custom PVT ranges", "properties": {"ranges": {"type": "object", "properties": {"kp": {"$ref": "#/definitions/Range"}, "kd": {"$ref": "#/definitions/Range"}, "position": {"$ref": "#/definitions/Range"}, "speed": {"$ref": "#/definitions/Range"}, "torque": {"$ref": "#/definitions/Range"}, "current": {"$ref": "#/definitions/Range"}, "kt": {"type": "number"}}, "required": ["kp", "kd", "position", "speed", "torque", "current", "kt"]}}, "required": ["ranges"]}
          ]
        }
      },
      "required": ["adapterId", "busId", "motorId", "initParam"]
    })";
    return oss.str();
}

/**
 * @brief DoubleCoupleJoint 标定参数 JSON Schema 定义
 */
inline const char* kCalibrationInputSchema = R"(    "CalibrationInput": {
      "type": "object",
      "properties": {
        "leftJointCenter": {"$ref": "#/definitions/Point3"},
        "rightJointCenter": {"$ref": "#/definitions/Point3"},
        "leftPlanarEndpoint": {"$ref": "#/definitions/Point3"},
        "rightPlanarEndpoint": {"$ref": "#/definitions/Point3"},
        "rollCenter": {"$ref": "#/definitions/Point3"},
        "pitchCenter": {"$ref": "#/definitions/Point3"},
        "leftSpatialEndpoint": {"$ref": "#/definitions/Point3"},
        "rightSpatialEndpoint": {"$ref": "#/definitions/Point3"},
        "rotateDirection": {"$ref": "#/definitions/Point3"},
        "rotationOrder": {"oneOf": [{"type": "string", "enum": ["PitchThenRoll", "RollThenPitch", "pitch_then_roll", "roll_then_pitch"]}, {"type": "integer", "enum": [0, 1]}]}
      },
      "required": ["leftJointCenter", "rightJointCenter", "leftPlanarEndpoint", "rightPlanarEndpoint", "rollCenter", "pitchCenter", "leftSpatialEndpoint", "rightSpatialEndpoint", "rotationOrder"]
    })";

/**
 * @brief 三维点 JSON Schema 定义
 */
inline const char* kPoint3Schema = R"(    "Point3": {
      "type": "object",
      "properties": {
        "x": {"type": "number"},
        "y": {"type": "number"},
        "z": {"type": "number"}
      },
      "required": ["x", "y", "z"]
    })";

/**
 * @brief 判断字段元数据数组中是否存在指定字段类型。
 * @tparam Fields 字段元数据数组引用。
 * @tparam Type 需要查找的字段类型。
 * @tparam Indices 参与展开的字段索引序列。
 * @return true 表示任一字段的类型等于 Type。
 */
template <const auto& Fields, FieldType Type, size_t... Indices>
inline constexpr bool HasFieldType(std::index_sequence<Indices...>) {
    return ((Fields[Indices].type == Type) || ... || false);
}

/**
 * @brief 判断字段元数据数组中是否存在指定字段类型。
 * @tparam Fields 字段元数据数组引用。
 * @tparam Type 需要查找的字段类型。
 * @return true 表示任一字段的类型等于 Type。
 */
template <const auto& Fields, FieldType Type>
inline constexpr bool HasFieldType() {
    return HasFieldType<Fields, Type>(std::make_index_sequence<Fields.size()>{});
}

/**
 * @brief 将 JSON 值赋给普通字段。
 * @tparam T 目标字段类型。
 * @param value JSON 字段值。
 * @param target 输出字段引用。
 */
template <typename T>
inline void AssignJsonField(const nlohmann::json& value, T& target) {
    target = value.get<T>();
}

/**
 * @brief 将 JSON 值赋给可选字段。
 * @tparam T 可选字段内部类型。
 * @param value JSON 字段值。
 * @param target 输出 optional 字段引用。
 */
template <typename T>
inline void AssignJsonField(const nlohmann::json& value, std::optional<T>& target) {
    target = value.get<T>();
}

/**
 * @brief 生成单个字段的JSON Schema
 * @param field 字段信息
 * @return JSON Schema字符串
 */
inline std::string GenerateFieldSchema(const FieldInfo& field) {
    std::ostringstream oss;
    oss << "\"" << field.name << "\": {";
    switch (field.type) {
        case FieldType::String:
            oss << "\"type\": \"string\"";
            break;
        case FieldType::Integer:
            oss << "\"type\": \"integer\"";
            break;
        case FieldType::Float:
            oss << "\"type\": \"number\"";
            break;
        case FieldType::Bool:
            oss << "\"type\": \"boolean\"";
            break;
        case FieldType::Range:
            oss << "\"$ref\": \"#/definitions/Range\"";
            break;
        case FieldType::Complex:
            oss << "\"$ref\": \"#/definitions/MotorParams\"";
            break;
        case FieldType::Axis:
            oss << "\"type\": \"string\", \"enum\": [\"roll\", \"pitch\", \"Roll\", \"Pitch\"]";
            break;
        case FieldType::Calibration:
            oss << "\"$ref\": \"#/definitions/CalibrationInput\"";
            break;
    }
    if (field.description && field.description[0] != '\0') {
        oss << ", \"description\": \"" << field.description << "\"";
    }
    oss << "}";
    return oss.str();
}

/**
 * @brief 生成完整的JSON Schema
 * @tparam Fields 字段信息数组
 * @param className 类名
 * @return 完整的JSON Schema字符串
 */
template <const auto& Fields>
inline std::string GenerateSchema(const char* className) {
    constexpr size_t kFieldCount = Fields.size();
    std::ostringstream oss;
    oss << "{\n  \"$schema\": \"http://json-schema.org/draft-07/schema#\",\n";
    oss << "  \"title\": \"" << className << " Config\",\n  \"type\": \"object\",\n";
    oss << "  \"definitions\": {\n" << kRangeSchema << ",\n" << GetMotorParamsSchema();
    if constexpr (HasFieldType<Fields, FieldType::Calibration>()) {
        oss << ",\n" << kPoint3Schema << ",\n" << kCalibrationInputSchema;
    }
    oss << "\n  },\n";
    oss << "  \"properties\": {\n";
    for (size_t i = 0; i < kFieldCount; ++i) {
        oss << "    " << GenerateFieldSchema(Fields[i]);
        if (i < kFieldCount - 1)
            oss << ",";
        oss << "\n";
    }
    oss << "  },\n  \"required\": [";
    bool first = true;
    for (size_t i = 0; i < kFieldCount; ++i) {
        if (Fields[i].required) {
            if (!first)
                oss << ", ";
            oss << "\"" << Fields[i].name << "\"";
            first = false;
        }
    }
    oss << "]\n}";
    return oss.str();
}

/// @cond INTERNAL

/**
 * @defgroup JsonConfigMacros JSON配置生成宏
 * @brief 使用Boost.Preprocessor生成配置结构体
 *
 * 用法示例：
 * @code
 * struct Config {
 *     GENERATE_CONFIG(RJoint,
 *         ((motorParams, MotorParams, Complex, true, "Motor initialization parameters"))
 *         ((limit, Range<float>, Range, true, "Position limit range"))
 *     )
 * };
 * @endcode
 * @{
 */

/// 辅助宏：从元组提取字段声明
#define _ENCOS_FIELD_DECL(r, data, elem) BOOST_PP_TUPLE_ELEM(1, elem) BOOST_PP_TUPLE_ELEM(0, elem);

/// 辅助宏：从元组提取FieldInfo
#define _ENCOS_FIELD_META(r, data, elem)                                                \
    encos::FieldInfo{BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(0, elem)),                   \
                                        encos::FieldType::BOOST_PP_TUPLE_ELEM(2, elem), \
                                        BOOST_PP_TUPLE_ELEM(3, elem),                   \
                                        BOOST_PP_TUPLE_ELEM(4, elem)},

/// 辅助宏：从JSON提取字段值
#define _ENCOS_FIELD_FROM_JSON(r, data, elem)                                         \
    if constexpr (BOOST_PP_TUPLE_ELEM(3, elem)) {                                     \
        encos::AssignJsonField(j.at(BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(0, elem))), \
                                    c.BOOST_PP_TUPLE_ELEM(0, elem));                  \
    } else {                                                                          \
        const auto it = j.find(BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(0, elem)));      \
        if (it != j.end()) {                                                          \
            encos::AssignJsonField(*it, c.BOOST_PP_TUPLE_ELEM(0, elem));              \
        }                                                                             \
    }

/**
 * @brief 生成配置结构体的主宏（带JSON支持）
 * @param className 类名
 * @param seq 字段序列，每个字段为 ((名称, 类型, FieldType, required, 描述))
 *
 * 生成以下成员：
 * - 字段声明
 * - kJointType 静态常量
 * - kFields 静态数组
 * - ToJsonSchema() 静态方法
 * - FromJson() 静态方法
 */
#define GENERATE_CONFIG(className, seq)                                                      \
    BOOST_PP_SEQ_FOR_EACH(_ENCOS_FIELD_DECL, , seq)                                          \
    static constexpr const char* kJointType = BOOST_PP_STRINGIZE(className);                 \
    inline static constexpr std::array<encos::FieldInfo, BOOST_PP_SEQ_SIZE(seq)> kFields = { \
        BOOST_PP_SEQ_FOR_EACH(_ENCOS_FIELD_META, , seq)};                                    \
    /**                                                                                      \
     * @brief 生成该关节配置结构体的 JSON Schema。                               \
     * @return JSON Schema 字符串。                                                      \
     */                                                                                      \
    static std::string ToJsonSchema() {                                                      \
        return encos::GenerateSchema<kFields>(BOOST_PP_STRINGIZE(className));                \
    }                                                                                        \
    /**                                                                                      \
     * @brief 从 JSON 对象解析该关节配置结构体。                               \
     * @param j 配置 JSON 对象。                                                        \
     * @return 解析后的 Config 实例。                                                 \
     */                                                                                      \
    static Config FromJson(const nlohmann::json& j) {                                        \
        Config c;                                                                            \
        BOOST_PP_SEQ_FOR_EACH(_ENCOS_FIELD_FROM_JSON, , seq)                                 \
        return c;                                                                            \
    }

/// @}
/// @endcond

}  // namespace encos
