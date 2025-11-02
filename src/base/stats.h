#ifndef     RAMULATOR_BASE_STATS_H
#define     RAMULATOR_BASE_STATS_H

#include <vector>
#include <string>
#include <variant>
#include <iostream>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "base/type.h"
#include "base/exception.h"

#include <cstdio>
#define YOAV(fmt...)				\
    do {					\
      	printf("# YOAV: " fmt);			\
	      printf("\n");				\
    } while(0)

namespace Ramulator {

template <typename Tret>
class BinOpBase {
    public:
    BinOpBase() {};
    virtual Tret run() const = 0;
};

template <typename Tret, typename T1, typename T2>
class BinOp : public BinOpBase<Tret> {
  private:
    inline static T1 T1_NULL = T1();
    inline static T2 T2_NULL = T2();
  protected:
    T1& _op1;
    T2& _op2;

  public:
    BinOp() : BinOpBase<Tret>(), _op1(T1_NULL), _op2(T2_NULL) {/* std::cerr<<"Error: BinOp stats must never be reset"<<std::endl<<std::stacktrace::current()<<std::endl; assert(0);*/}
    BinOp(T1& op1, T2& op2) : BinOpBase<Tret>(), _op1(op1), _op2(op2) {}

    BinOp<Tret, T1, T2>& operator=(const BinOp<Tret, T1, T2>& oth) {
      this->_op1 = oth._op1;
      this->_op2 = oth._op2;
      return *this;
    };

    virtual Tret run() const = 0;
    Tret operator*() const { return this->run(); };
    explicit operator Tret() const {
        return this->run();
    }};

template <typename Tret, typename T1, typename T2>
inline YAML::Emitter& operator<<(YAML::Emitter& emitter,
                                   const BinOp<Tret, T1, T2>& binop) {
  Tret val = binop.run();
  emitter << val;
  return emitter;
}

template <typename T1, typename T2>
class ADDi : public BinOp<uint64_t, T1, T2>{
  public:
    ADDi() : BinOp<uint64_t, T1, T2>() {}
    ADDi(T1& op1, T2& op2) : BinOp<uint64_t, T1, T2>(op1, op2) {};
    uint64_t run() const {
      return static_cast<uint64_t>(this->_op1) + static_cast<uint64_t>(this->_op2);
    }
};

template <typename T1, typename T2>
class DIV : public BinOp<double, T1, T2> {
  public:
    DIV() : BinOp<double, T1, T2>() {}
    DIV(T1& op1, T2& op2) : BinOp<double, T1, T2>(op1, op2) {};
    double run() const {

      return static_cast<double>(this->_op1) / static_cast<double>(static_cast<uint64_t>(this->_op2));
    }
};

class Implementation;
class StatWrapperBase {
  public:
    virtual void emit_to(YAML::Emitter& emitter) = 0;
    virtual void reset(void) = 0;
};

template<typename T>
class StatWrapper;

class Stats {
  template<typename T>
  friend class StatWrapper;
  friend YAML::Emitter& operator << (YAML::Emitter& emitter, const Stats& s);

  private:
    Registry_t<StatWrapperBase*> _registry;

  public:
    bool is_empty() {
      return _registry.size() == 0;
    }

    void reset() {
      for (auto [stat_name, stat_ptr] : _registry) {
        stat_ptr->reset();
      }
    }
};


template<typename T>
class StatWrapper : public StatWrapperBase {
  // static_assert(std::is_arithmetic_v<T>, "Only arithmetic types are allowed for Statistics!");

  private:
    std::variant<BinOpBase<T>*, T*, std::vector<T>*> _ref;
    std::string _name;
    std::string _desc;
    bool _need_reset;

    const Implementation& _impl;
    Stats& _stats;

  public:
    StatWrapper(T& val, const Implementation& impl, Stats& stats, bool need_reset=true) : _ref(&val), _impl(impl), _stats(stats), _need_reset(need_reset) {};
    StatWrapper(std::vector<T>& val, const Implementation& impl, Stats& stats, bool need_reset=true) : _ref(&val), _impl(impl), _stats(stats), _need_reset(need_reset){};
    StatWrapper(BinOpBase<T>& binop, const Implementation& impl, Stats& stats, bool need_reset=true) : _ref(binop), _impl(impl), _stats(stats), _need_reset(need_reset) {};

    StatWrapper& name(std::string name) {
	_name = name;
      if (auto it = _stats._registry.find(name); it != _stats._registry.end()) {
        throw ConfigurationError("Stat {} of implementation is already registered!", name);
      }
      _stats._registry[name] = this;
      return *this;
    };
    template <typename... Args>
    StatWrapper& name(fmt::format_string<Args...> format_str, Args&&... args) {
      return name(fmt::format(format_str, std::forward<Args>(args)...));
    };

    StatWrapper& desc(std::string desc) { _desc = desc; return *this; };

    void emit_to(YAML::Emitter& emitter) override {

      if        (std::holds_alternative<T*>(_ref)) {
        emitter << YAML::Key << _name;
        auto val = *std::get<T*>(_ref);
        emitter << YAML::Value << val;
        if (!_desc.empty()) {
          emitter << YAML::Comment(_desc);
        }
      } else if (std::holds_alternative<std::vector<T>*>(_ref)) {
        emitter << YAML::Key << _name;
        if (!_desc.empty()) {
          emitter << YAML::Comment(_desc);
        }
        emitter << YAML::Value <<  YAML::BeginSeq;
        for (const auto _val : *(std::get<std::vector<T>*>(_ref))) {
          emitter << _val;
        }
        emitter << YAML::EndSeq;
      }

    };

    void reset(void) {
      if (!_need_reset) {
        return;
      }

      if        (std::holds_alternative<T*>(_ref)) {
        *std::get<T*>(_ref) = T();
      }
      else if (std::holds_alternative<std::vector<T>*>(_ref)) {
        for (auto& _val : *(std::get<std::vector<T>*>(_ref))) {
          _val = T();
        }
      }
    }
};

}        // namespace Ramulator


#endif   // RAMULATOR_BASE_STATS_H
