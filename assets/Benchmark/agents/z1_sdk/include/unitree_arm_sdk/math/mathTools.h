#ifndef MATHTOOLS_H
#define MATHTOOLS_H

#include <iostream>
#include <math.h>
#include "unitree_arm_sdk/math/mathTypes.h"

namespace UNITREE_ARM {

template<typename T>
T max(T value){
    return value;
}

template<typename T, typename... Args>
inline T max(const T val0, const Args... restVal){
    return val0 > (max<T>(restVal...)) ? val0 : max<T>(restVal...);
}

template<typename T>
T min(T value){
    return value;
}

template<typename T, typename... Args>
inline T min(const T val0, const Args... restVal){
    return val0 > (min<T>(restVal...)) ? val0 : min<T>(restVal...);
}

enum class TurnDirection{
    NOMATTER,
    POSITIVE,
    NEGATIVE
};

/* first - second */
inline double angleError(double first, double second, TurnDirection direction = TurnDirection::NOMATTER){
    double firstMod = fmod(first, 2.0*M_PI);
    double secondMod = fmod(second, 2.0*M_PI);

    if(direction == TurnDirection::POSITIVE){
        if(firstMod - secondMod < 0.0){
            return 2*M_PI + firstMod - secondMod;
        }else{
            return firstMod - secondMod;
        }
    }
    else if(direction == TurnDirection::NEGATIVE){
        if(firstMod - secondMod > 0.0){
            return -2*M_PI + firstMod - secondMod;
        }else{
            return firstMod - secondMod;
        }
    }else{//no matter
        if(fabs(firstMod - secondMod) > fabs(secondMod - firstMod)){
            return secondMod - firstMod;
        }else{
            return firstMod - secondMod;
        }        
    }
}

/* firstVec - secondVec */
inline VecX angleError(VecX firstVec, VecX secondVec, TurnDirection directionMatter = TurnDirection::NOMATTER){
    if(firstVec.rows() != secondVec.rows()){
        std::cout << "[ERROR] angleError, the sizes of firstVec and secondVec are different!" << std::endl;
    }

    VecX result = firstVec;
    for(int i(0); i<firstVec.rows(); ++i){
        result(i) = angleError(firstVec(i), secondVec(i), directionMatter);
    }

    return result;
}

inline bool vectorEqual(VecX v1, VecX v2, double tolerance){
    if(v1.rows() != v2.rows()){
        std::cout << "[WARNING] vectorEqual, the size of two vectors is not equal, v1 is "
            << v1.rows() << ", v2 is " << v2.rows() << std::endl;
        return false;
    }
    for(int i(0); i<v1.rows(); ++i){
        if(fabs(v1(i)-v2(i))>tolerance){
            return false;
        }
    }
    return true;
}

inline bool inInterval(double value, double limValue1, double limValue2, bool canEqual1 = false, bool canEqual2 = false){
    double lowLim, highLim;
    bool lowEqual, highEqual;
    if(limValue1 >= limValue2){
        highLim   = limValue1;
        highEqual = canEqual1;
        lowLim    = limValue2;
        lowEqual  = canEqual2;
    }else{
        lowLim    = limValue1;
        lowEqual  = canEqual1;
        highLim   = limValue2;
        highEqual = canEqual2;
    }

    if((value > highLim) || (value < lowLim)){
        return false;
    }
    if((value == highLim) && !highEqual){
        return false;
    }
    if((value == lowLim) && !lowEqual){
        return false;
    }

    return true;
}

inline double saturation(const double a, double limValue1, double limValue2){
    double lowLim, highLim;
    if(limValue1 >= limValue2){
        lowLim = limValue2;
        highLim= limValue1;
    }else{
        lowLim = limValue1;
        highLim= limValue2;
    }

    if(a < lowLim){
        return lowLim;
    }else if(a > highLim){
        return highLim;
    }else{
        return a;
    }
}

inline double saturation(const double a, Vec2 limits){
    return saturation(a, limits(0), limits(1));
}

template<typename T0, typename T1>
inline T0 killZeroOffset(T0 a, const T1 limit){
    if((a > -limit) && (a < limit)){
        a = 0;
    }
    return a;
}

template<typename T0, typename T1, typename T2>
inline T1 invNormalize(const T0 value, const T1 min, const T2 max, const double minLim = -1, const double maxLim = 1){
	return (value-minLim)*(max-min)/(maxLim-minLim) + min;
}

// x: [0, 1], windowRatio: (0, 0.5)
template<typename T>
inline T windowFunc(const T x, const T windowRatio, const T xRange=1.0, const T yRange=1.0){
    if((x < 0)||(x > xRange)){
        std::cout << "[ERROR][windowFunc] The x=" << x << ", which should between [0, xRange]" << std::endl;
    }
    if((windowRatio <= 0)||(windowRatio >= 0.5)){
        std::cout << "[ERROR][windowFunc] The windowRatio=" << windowRatio << ", which should between [0, 0.5]" << std::endl;
    }

    if(x/xRange < windowRatio){
        return x * yRange / (xRange * windowRatio);
    }else if(x/xRange > 1 - windowRatio){
        return yRange * (xRange - x)/(xRange * windowRatio);
    }else{
        return yRange;
    }
}

template<typename T1, typename T2>
inline void updateAverage(T1 &exp, T2 newValue, double n){
    if(exp.rows()!=newValue.rows()){
        std::cout << "The size of updateAverage is error" << std::endl;
        exit(-1);
    }
    if(fabs(n - 1) < 0.001){
        exp = newValue;
    }else{
        exp = exp + (newValue - exp)/n;
    }
}

template<typename T1, typename T2, typename T3>
inline void updateCovariance(T1 &cov, T2 expPast, T3 newValue, double n){
    if( (cov.rows()!=cov.cols()) || (cov.rows() != expPast.rows()) || (expPast.rows()!=newValue.rows())){
        std::cout << "The size of updateCovariance is error" << std::endl;
        exit(-1);
    }
    if(fabs(n - 1) < 0.1){
        cov.setZero();
    }else{
        cov = cov*(n-1)/n + (newValue-expPast)*(newValue-expPast).transpose()*(n-1)/(n*n);
    }
}

template<typename T1, typename T2, typename T3>
inline void updateAvgCov(T1 &cov, T2 &exp, T3 newValue, double n){
    // The order matters!!! covariance first!!!
    updateCovariance(cov, exp, newValue, n);
    updateAverage(exp, newValue, n);
}

// Calculate average value and covariance
class AvgCov{
public:
    AvgCov(unsigned int size, std::string name, bool avgOnly=false, unsigned int showPeriod=1000, unsigned int waitCount=5000, double zoomFactor=10000)
            :_size(size), _showPeriod(showPeriod), _waitCount(waitCount), _zoomFactor(zoomFactor), _valueName(name), _avgOnly(avgOnly) {
        _exp.resize(size);
        _cov.resize(size, size);
        _defaultWeight.resize(size, size);
        _defaultWeight.setIdentity();
        _measureCount = 0;
    }
    void measure(VecX newValue){
        ++_measureCount;

        if(_measureCount > _waitCount){
            updateAvgCov(_cov, _exp, newValue, _measureCount-_waitCount);
            if(_measureCount % _showPeriod == 0){
            // if(_measureCount < _waitCount + 5){
                std::cout << "******" << _valueName << " measured count: " << _measureCount-_waitCount << "******" << std::endl;
                // std::cout << _zoomFactor << " Times newValue of " << _valueName << std::endl << (_zoomFactor*newValue).transpose() << std::endl;
                std::cout << _zoomFactor << " Times Average of " << _valueName << std::endl << (_zoomFactor*_exp).transpose() << std::endl;
                if(!_avgOnly){
                    std::cout << _zoomFactor << " Times Covariance of " << _valueName << std::endl << _zoomFactor*_cov << std::endl;
                }
            }
        }
    }
private:
    VecX _exp;
    MatX _cov;
    MatX _defaultWeight;
    bool _avgOnly;
    unsigned int _size;
    unsigned int _measureCount;
    unsigned int _showPeriod;
    unsigned int _waitCount;
    double _zoomFactor;
    std::string _valueName;
};
}
#endif