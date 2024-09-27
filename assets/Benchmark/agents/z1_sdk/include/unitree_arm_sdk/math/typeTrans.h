#ifndef TYPETRANS_H
#define TYPETRANS_H

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>

namespace UNITREE_ARM {
namespace typeTrans{

inline void addValue(std::vector<double> &vec, double value){
    vec.push_back(value);
}

inline double getValue(std::vector<double> &vec, double value){
    value = vec.at(0);
    std::vector<double>::iterator begin = vec.begin();
    vec.erase(begin);

    return value;
}

inline void addValue(std::vector<double> &vec, Eigen::MatrixXd value){
    for(int i(0); i<value.rows(); ++i){
        for(int j(0); j<value.cols(); ++j){
            vec.push_back(value(i, j));
        }
    }
}

inline Eigen::MatrixXd getValue(std::vector<double> &vec, Eigen::MatrixXd value){
    std::vector<double>::iterator begin = vec.begin();
    std::vector<double>::iterator end = begin;

    for(int i(0); i<value.rows(); ++i){
        for(int j(0); j<value.cols(); ++j){
            value(i, j) = *end;
            ++end;
        }
    }

    vec.erase(begin, end);

    return value;
}

/* combine different type variables to vector */
template<typename T>
inline void combineToVector(std::vector<double> &vec, T value){
    addValue(vec, value);
}

template<typename T, typename... Args>
inline void combineToVector(std::vector<double> &vec, const T t, const Args... rest){
    combineToVector(vec, t);

    combineToVector(vec, rest...);
}


/* extract different type variables from vector */
template<typename T>
inline void extractVector(std::vector<double> &vec, T &value){
    value = getValue(vec, value);
}

template<typename T, typename... Args>
inline void extractVector(std::vector<double> &vec, T &t, Args&... rest){
    extractVector(vec, t);

    extractVector(vec, rest...);
}

}
}
#endif  // TYPETRANS_H