#pragma once

#include <sstream>

class ErrorWrapper {
public:
    template<typename... Args>
    ErrorWrapper(const std::string &msg, bool active, const char* file, int line, Args... args)
        : _active{active} {
        if (!_active)
            return;
        _stream << file << ", " << line << ". ";
        generateMessage(msg, args...);
    }

    template<typename... Args>
    ErrorWrapper(bool active, const char* file, int line, Args... args)
        : ErrorWrapper("Condition is not met", active, file, line, args...) {        
    }

    std::ostream& operator()() { return _stream; }

    ~ErrorWrapper() noexcept(false) {
        if (_active)
            throw std::runtime_error(_stream.str());
    }
private:
    void message(){}

    template<typename T, typename... Args>
    void message(T opt, Args... args) {
        _stream << opt;
        message(args...);
    }

    template<typename... Args>
    void generateMessage(const std::string &base, Args... args) {
        _stream << base << ": ";
        message(args...);
    }

 std::stringstream _stream;
 const bool _active;
};

#ifdef CHECK
#define GLOG_CHECK CHECK
#undef CHECK
#endif
#define CHECK(condition) ErrorWrapper{!(condition), __FILE__, __LINE__, #condition}()

template <typename T>
T CheckNotNull(const std::string& base_message, const char* file, int line, T&& val) {
    if (val == nullptr) {
        ErrorWrapper{base_message, true, file, line};
    }
    return std::forward<T>(val);
}

#ifdef CHECK_NOTNULL
#define GLOG_CHECK_NOTNULL CHECK_NOTNULL
#undef CHECK_NOTNULL
#endif
#define CHECK_NOTNULL(val) CheckNotNull(#val " mustn't be null", __FILE__, __LINE__, val)
// #define CHECK_NOTNULL(val) ErrorWrapper{#val" mustn't be null", !(val == nullptr), __FILE__, __LINE__, #val}()

#define CHECK_OPERATION(op, val1, val2) ErrorWrapper{!(val1 op val2), __FILE__, __LINE__, val1, #op, val2}()

#ifdef CHECK_EQ
#define GLOG_CHECK_EQ CHECK_EQ
#undef CHECK_EQ
#endif
#define CHECK_EQ(val1, val2) CHECK_OPERATION(==, val1, val2)

#ifdef CHECK_NE
#define GLOG_CHECK_NE CHECK_NE
#undef CHECK_NE
#endif
#define CHECK_NE(val1, val2) CHECK_OPERATION(!=, val1, val2)

#ifdef CHECK_LE
#define GLOG_CHECK_LE CHECK_LE
#undef CHECK_LE
#endif
#define CHECK_LE(val1, val2) CHECK_OPERATION(<=, val1, val2)

#ifdef CHECK_LT
#define GLOG_CHECK_LT CHECK_LT
#undef CHECK_LT
#endif
#define CHECK_LT(val1, val2) CHECK_OPERATION(< , val1, val2)

#ifdef CHECK_GE
#define GLOG_CHECK_GE CHECK_GE
#undef CHECK_GE
#endif
#define CHECK_GE(val1, val2) CHECK_OPERATION(>=, val1, val2)

#ifdef CHECK_GT
#define GLOG_CHECK_GT CHECK_GT
#undef CHECK_GT
#endif
#define CHECK_GT(val1, val2) CHECK_OPERATION(> , val1, val2)
