#pragma once

#include <Eigen/Core>
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <mutex>
#include <sstream>
#include <string>

#include <ros/ros.h>

namespace decision_trace {

struct Field {
  std::string key;
  std::string value;
  bool raw{false};
};

inline std::string escape(const std::string& s) {
  std::ostringstream out;
  for (char c : s) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          out << "\\u00";
          const char* hex = "0123456789abcdef";
          out << hex[(c >> 4) & 0x0f] << hex[c & 0x0f];
        } else {
          out << c;
        }
    }
  }
  return out.str();
}

inline Field str(const std::string& key, const std::string& value) {
  return Field{key, "\"" + escape(value) + "\"", true};
}

inline Field raw(const std::string& key, const std::string& value) {
  return Field{key, value, true};
}

inline Field num(const std::string& key, int value) {
  return Field{key, std::to_string(value), true};
}

inline Field num(const std::string& key, unsigned int value) {
  return Field{key, std::to_string(value), true};
}

inline Field num(const std::string& key, uint8_t value) {
  return Field{key, std::to_string(static_cast<unsigned int>(value)), true};
}

inline Field num(const std::string& key, double value) {
  std::ostringstream ss;
  ss << value;
  return Field{key, ss.str(), true};
}

inline Field boolean(const std::string& key, bool value) {
  return Field{key, value ? "true" : "false", true};
}

inline Field vec3(const std::string& key, const Eigen::Vector3d& v) {
  std::ostringstream ss;
  ss << "[" << v.x() << "," << v.y() << "," << v.z() << "]";
  return Field{key, ss.str(), true};
}

inline bool enabled() {
  const char* env = std::getenv("TRACE_ENABLE");
  return env != nullptr &&
         (std::string(env) == "1" || std::string(env) == "true" ||
          std::string(env) == "TRUE" || std::string(env) == "on");
}

inline std::string traceId() {
  const char* env = std::getenv("TRACE_ID");
  return env != nullptr ? std::string(env) : std::string();
}

inline std::string traceFile() {
  const char* file = std::getenv("DECISION_TRACE_FILE");
  if (file != nullptr && file[0] != '\0') {
    return std::string(file);
  }
  const char* dir = std::getenv("TRACE_DIR");
  if (dir != nullptr && dir[0] != '\0') {
    return std::string(dir) + "/decision.jsonl";
  }
  return std::string();
}

inline void log(const std::string& module, const std::string& event,
                std::initializer_list<Field> fields = {}) {
  if (!enabled()) {
    return;
  }

  const std::string path = traceFile();
  if (path.empty()) {
    return;
  }

  static std::mutex mu;
  static std::atomic<unsigned long long> seq{0};
  std::lock_guard<std::mutex> lock(mu);

  std::ofstream out(path.c_str(), std::ios::out | std::ios::app);
  if (!out.is_open()) {
    return;
  }

  out << "{"
      << "\"trace_id\":\"" << escape(traceId()) << "\","
      << "\"seq\":" << seq.fetch_add(1) << ","
      << "\"stamp\":" << ros::Time::now().toSec() << ","
      << "\"node\":\"" << escape(ros::this_node::getName()) << "\","
      << "\"module\":\"" << escape(module) << "\","
      << "\"event\":\"" << escape(event) << "\"";

  for (const auto& f : fields) {
    out << ",\"" << escape(f.key) << "\":" << f.value;
  }
  out << "}\n";
}

}  // namespace decision_trace
