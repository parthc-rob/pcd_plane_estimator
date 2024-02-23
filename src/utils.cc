#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <ctime>

namespace utils {
    bool getEnvVar( std::string const & key )
    {
        char * val = getenv( key.c_str() );
        return val == NULL ? false : bool(val);
    }

    class DebugStream {
    public:
        DebugStream() : enabled(getEnvVar("DEBUG")) {}

    template <typename T>
        DebugStream& operator<<(const T& value) {
            if (enabled) {
                output_stream << value;
            }
            return *this;
        }

        // Overload for manipulators like std::endl
        DebugStream& operator<<(std::ostream& (*manipulator)(std::ostream&)) {
            if (enabled) {
                auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch()
                ).count() % 1000;

                std::tm local_tm = {};
                localtime_r(&now, &local_tm);

                output_stream << manipulator;
                std::cout << "\n[" << std::put_time(&local_tm, "%Y-%m-%d:%H:%M:%S.") << std::setw(6) << std::setfill('0') << milliseconds << "] DEBUG: " << output_stream.str();
                output_stream.str("");
            }
            return *this;
        }

    private:
        const bool enabled;
        std::ostringstream output_stream;
    };

}
