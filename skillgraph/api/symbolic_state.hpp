#pragma once

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <regex>

namespace skillgraph {

/**
 * @brief Simple predicate proxy that acts like a boolean attribute
 */
class PredicateProxy {
public:
    PredicateProxy(std::unordered_map<std::string, bool>& predicates, const std::string& name) 
        : predicates_(predicates), name_(name) {}
    
    operator bool() const { return predicates_[name_]; }
    PredicateProxy& operator=(bool value) { predicates_[name_] = value; return *this; }
    
private:
    std::unordered_map<std::string, bool>& predicates_;
    std::string name_;
};

/**
 * @brief Dynamic PDDL state with attribute-like access to predicates
 */
class pddl_state {
public:
    pddl_state() = default;
    explicit pddl_state(const std::string& domain_file_path) { loadDomain(domain_file_path); }
    
    bool loadDomain(const std::string& domain_file_path);
    
    // Dynamic attribute access - returns a proxy that acts like a boolean
    PredicateProxy operator[](const std::string& predicate_name) {
        return PredicateProxy(predicates_, predicate_name);
    }
    
    bool operator[](const std::string& predicate_name) const {
        auto it = predicates_.find(predicate_name);
        return it != predicates_.end() ? it->second : false;
    }

private:
    bool parseDomainFile(const std::string& content);
    
    std::unordered_map<std::string, bool> predicates_;
    std::vector<std::string> predicate_names_;
};

typedef std::shared_ptr<pddl_state> pddl_state_ptr;

}  // namespace skillgraph

