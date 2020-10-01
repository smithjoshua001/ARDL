#pragma once
#include <iostream>
#include <fstream>
#include <variant>
#include "ARDL/typedefs.hpp"
#include <fmt/format.h>
namespace ARDL {
    namespace Util {
        namespace BufferLogging {
            class CSV {
                 private:
                std::string filename;
                std::map<std::string, std::vector<std::string>> data;
                std::vector<std::string> fields;
                std::ofstream ofs;
                size_t bufferSize;

                 public:
                CSV(std::string filename, size_t bufferSize): filename(filename), bufferSize(bufferSize) {
                    ofs= std::ofstream(filename, std::ofstream::out);
                }

                void addField(std::string name) {
                    data[name]= std::vector<std::string>();
                    fields.push_back(name);
                }

                template<typename T, typename std::enable_if_t<std::is_arithmetic<T>::value> * = nullptr>
                void addData(std::string name, T input) {
                    if(!data.count(name)) { fields.push_back(name); }
                    data[name].push_back(std::to_string(input));
                }
                void addData(std::string name, std::string input) { data[name].push_back(input); }
                void addField(std::string format, size_t size) {
                    for(size_t i= 0; i < size; i++) {
                        data[fmt::format(format, i)]= std::vector<std::string>();
                        fields.push_back(fmt::format(format, i));
                    }
                }
                template<typename T>
                void addData(std::string format, const Eigen::EigenBase<T> &data) {
                    for(size_t i= 0; i < data.size(); i++) { addData(fmt::format(format, i), data.derived()(i)); }
                }
                void saveFields() {
                    for(size_t j= 0; j < fields.size() - 1; j++) { ofs << fields[j] << ","; }
                    ofs << fields.back() << std::endl;
                    ofs.flush();
                }
                void save() {
                    for(size_t i= 0; i < data[fields[0]].size(); i++) {
                        for(size_t j= 0; j < fields.size() - 1; j++) { ofs << data[fields[j]][i] << ","; }
                        ofs << data[fields.back()][i] << std::endl;
                    }
                    ofs.flush();
                }

                void clear() {
                    for(size_t j= 0; j < fields.size(); j++) { data[fields[j]].clear(); }
                }

                void saveAndClear() {
                    if(data[fields[0]].size() >= bufferSize) {
                        save();
                        clear();
                    }
                }

                void close() {
                    ofs.flush();
                    ofs.close();
                }
            };
        } // namespace BufferLogging
    }     // namespace Util
} // namespace ARDL
