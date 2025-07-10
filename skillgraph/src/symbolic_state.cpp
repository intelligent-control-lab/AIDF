#include "symbolic_state.hpp"
#include <iostream>
#include <regex>

namespace skillgraph {

bool pddl_state::loadDomain(const std::string& domain_file_path) {
    std::ifstream file(domain_file_path);
    if (!file.is_open()) {
        std::cerr << "Error: Cannot open domain file: " << domain_file_path << std::endl;
        return false;
    }
    
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    file.close();
    
    return parseDomainFile(content);
}

bool pddl_state::parseDomainFile(const std::string& content) {
    // Clear existing predicates
    predicates_.clear();
    predicate_names_.clear();

    // Manually find the :predicates section
    size_t start = content.find("(:predicates");
    if (start == std::string::npos) {
        std::cerr << "Error: No predicates section found in domain file" << std::endl;
        return false;
    }

    // Track parentheses to extract the full block
    int paren_count = 0;
    bool started = false;
    size_t end = start;

    for (; end < content.size(); ++end) {
        if (content[end] == '(') {
            paren_count++;
            started = true;
        } else if (content[end] == ')') {
            paren_count--;
            if (paren_count == 0 && started) {
                ++end; // include the closing ')'
                break;
            }
        }
    }

    if (paren_count != 0) {
        std::cerr << "Error: Unbalanced parentheses in :predicates section" << std::endl;
        return false;
    }

    std::string predicates_section = content.substr(start, end - start);

    // Regular expression to find individual predicate definitions
    // Matches patterns like: (predicate_name ?param1 - type1 ?param2 - type2)
    std::regex predicate_regex(R"(\(\s*([a-zA-Z_][a-zA-Z0-9_-]*)\s*([^)]*)\))");
    std::sregex_iterator predicate_iter(predicates_section.begin(), predicates_section.end(), predicate_regex);
    std::sregex_iterator predicate_end;

    for (; predicate_iter != predicate_end; ++predicate_iter) {
        std::string predicate_name = (*predicate_iter)[1].str();
        std::string parameters_str = (*predicate_iter)[2].str();

        // Store predicate name and initialize to false
        predicate_names_.push_back(predicate_name);
        predicates_[predicate_name] = false;

        std::cout << "Found predicate: " << predicate_name << std::endl;
    }

    if (predicate_names_.empty()) {
        std::cerr << "Warning: No predicates found in domain file" << std::endl;
        return false;
    }

    std::cout << "Successfully loaded " << predicate_names_.size() << " predicates from domain file" << std::endl;
    return true;
}


}  // namespace skillgraph
