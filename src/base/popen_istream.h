#include <iostream>
#include <istream>
#include <cstdio>
#include <stdexcept>
#include <cstring>

#include "base/exception.h"

namespace Ramulator {

class PopenIstream : public std::istream {
public:
    PopenIstream(const std::string& command, const std::string& mode = "r")
        : std::istream(&m_buffer), m_file(popen(command.c_str(), mode.c_str())) {
        if (!m_file) {
            throw ConfigurationError("popen failed on command {}", command);
        }
    }

    ~PopenIstream() override {
        if (m_file) {
            pclose(m_file);
        }
    }

    void close() {
        if (m_file) {
            pclose(m_file);
        }
        m_file = nullptr;
    }

private:
    class PopenBuffer : public std::streambuf {
    public:
        PopenBuffer(FILE* file) : m_file(file) {}

    protected:
        int underflow() override {
            if (!m_file) {
                return std::streambuf::traits_type::eof();
            }

            ssize_t bytesRead = fread(m_buffer.data(), 1, m_buffer.size(), m_file);

            if (bytesRead <= 0) {
                if(feof(m_file)){
                    return std::streambuf::traits_type::eof();
                }
                else{
                    return std::streambuf::traits_type::eof(); // Error case, treat as EOF
                }

            }

            setg(m_buffer.data(), m_buffer.data(), m_buffer.data() + bytesRead);
            return traits_type::to_int_type(m_buffer[0]);
        }

    private:
        FILE* m_file;
        std::array<char, 1024> m_buffer; // Adjust buffer size as needed
    };

    FILE* m_file;
    PopenBuffer m_buffer{m_file};
};

}; // namespace
