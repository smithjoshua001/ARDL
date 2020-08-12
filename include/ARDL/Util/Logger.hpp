#pragma once

#include <fmt/core.h>
#include <fmt/ostream.h>

#include <spdlog/spdlog.h>
#include "spdlog/sinks/stdout_color_sinks.h"
#include <string>

namespace ARDL {
    namespace Util {
        /**
         * @brief Wrapper for spdlog (Used for zero cost logging removal in release)
         *
         */
        class Logger {
        private:
            std::shared_ptr<spdlog::logger> console;
        public:
            /**
             * @brief Construct a new Logger object
             *
             * @param name Name of logger to connect to
             */
            Logger(std::string name) {
                console = spdlog::get(name);
                if (!console) {
                    console = spdlog::stdout_color_mt(name);
                }
                #ifdef LOG_DEBUG_ON
                    spdlog::set_level(spdlog::level::debug);
                #endif
            }
            /**
             * @brief Get the Console object
             *
             * @return std::shared_ptr<spdlog::logger>
             */
            std::shared_ptr<spdlog::logger> getConsole() {
                return console;
            }

            /**
             * @brief Checks whether to include DEBUG logging through the LOG_DEBUG_ON flag from CMAKE, determines which levels to use otherwise it disables the call by replacing the call with (void)0 which gets optimized out of the release code
             * TODO add throttled logging (through passing back and forth timestamps)
             */
            #ifdef LOG_DEBUG_ON
                #if LOG_DEBUG_ON >= 1
        #define LOG_DEBUG_LEVEL1(INPUT, ...) (ARDL::Util::RUNTIME_LOG_LEVEL < 1) ? (void)0 : (void)ARDL::Util::logger->getConsole()->debug((std::string("\033[33m[LEVEL1]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
        #define LOG_DEBUG_LEVEL1(INPUT, ...) (void)0
                #endif
                #if LOG_DEBUG_ON >= 2
        #define LOG_DEBUG_LEVEL2(INPUT, ...) (ARDL::Util::RUNTIME_LOG_LEVEL < 2) ? (void)0 : (void)ARDL::Util::logger->getConsole()->debug((std::string("\033[33m[LEVEL2]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
        #define LOG_DEBUG_LEVEL2(INPUT, ...) (void)0
                #endif
                #if LOG_DEBUG_ON >= 3
        #define LOG_DEBUG_LEVEL3(INPUT, ...) (ARDL::Util::RUNTIME_LOG_LEVEL < 3) ? (void)0 : (void)ARDL::Util::logger->getConsole()->debug((std::string("\033[33m[LEVEL3]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
        #define LOG_DEBUG_LEVEL3(INPUT, ...) (void)0
                #endif
                #if LOG_DEBUG_ON >= 4
        #define LOG_DEBUG_LEVEL4(INPUT, ...) (ARDL::Util::RUNTIME_LOG_LEVEL < 4) ? (void)0 : (void)ARDL::Util::logger->getConsole()->debug((std::string("\033[33m[LEVEL4]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
        #define LOG_DEBUG_LEVEL4(INPUT, ...) (void)0
                #endif
                #if LOG_DEBUG_ON >= 5
        #define LOG_DEBUG_LEVEL5(INPUT, ...) (ARDL::Util::RUNTIME_LOG_LEVEL < 5) ? (void)0 : (void)ARDL::Util::logger->getConsole()->debug((std::string("\033[33m[LEVEL5]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
        #define LOG_DEBUG_LEVEL5(INPUT, ...) (void)0
                #endif
                #if LOG_DEBUG_ON >= 6
        #define LOG_DEBUG_LEVEL6(INPUT, ...) (ARDL::Util::RUNTIME_LOG_LEVEL < 6) ? (void)0 : (void)ARDL::Util::logger->getConsole()->debug((std::string("\033[33m[LEVEL6]\033[0m ") + INPUT).c_str(), ## __VA_ARGS__)
                #else
        #define LOG_DEBUG_LEVEL6(INPUT, ...) (void)0
                #endif
            #else
    #define LOG_DEBUG_LEVEL1(INPUT, ...) (void)0
    #define LOG_DEBUG_LEVEL2(INPUT, ...) (void)0
    #define LOG_DEBUG_LEVEL3(INPUT, ...) (void)0
    #define LOG_DEBUG_LEVEL4(INPUT, ...) (void)0
    #define LOG_DEBUG_LEVEL5(INPUT, ...) (void)0
    #define LOG_DEBUG_LEVEL6(INPUT, ...) (void)0
            #endif
        };

        /**
         * @brief The global log object.
         *
         */
        static std::shared_ptr<Logger> logger = std::make_shared<Logger>("logger");

        #ifdef LOG_DEBUG_ON
            /**
             * If logging is enabled a runtime log level can also be used to limit the messages displayed. The level is determined by RUNTIME_LOG_LEVEL.
             */
            static size_t RUNTIME_LOG_LEVEL = LOG_DEBUG_ON;
        #endif
    }
}
